"""Explicit-lifetime, immutable-input preparation for repeated FAB cell queries."""

from threading import RLock
from uuid import uuid4
import math

from compas_robots.model import Joint
from compas_forge import (
    _compas_fab_collision_links, _combined_model_collision_mesh,
    _combined_rigid_body_collision_mesh, _robot_semantic_disabled_collision_pairs,
    _robot_link_pose_map, _configuration_from_trajectory_point,
    _require_valid_compas_fab_trajectory, _trajectory_times, _collision_timing,
    _interpolate_configuration, pose_from_frame, register_mesh_to_cache,
    unregister_mesh, check_swept_collision_cached_batch_poses,
    verify_clearance_cached_batch_poses,
    _coerce_pose,
)


def _interpolate_scene_pose(start, end, t):
    from compas.geometry import Quaternion
    q = Quaternion(start[6],*start[3:6]).slerp(Quaternion(end[6],*end[3:6]),t)
    return [a+(b-a)*t for a,b in zip(start[:3],end[:3])] + [q.x,q.y,q.z,q.w]


def _validate_full_configuration(model, configuration):
    """Validate the complete FK input, including values retained by partial paths."""
    for name,value,kind in zip(configuration.joint_names,configuration.joint_values,
                               configuration.joint_types):
        joint = model.get_joint_by_name(name)
        if joint is None or joint.type != kind:
            raise ValueError(f'invalid full configuration joint/type: {name}')
        if not math.isfinite(value):
            raise ValueError(f'nonfinite retained/full configuration value: {name}')
        if joint.type in (Joint.REVOLUTE,Joint.PRISMATIC) and joint.limit is not None:
            if value < joint.limit.lower or value > joint.limit.upper:
                raise ValueError(f'retained/full configuration outside joint limit: {name}')


class PreparedRobotCell:
    """Own a copied cell/state and retained native meshes until close().

    Changing the caller's geometry/state does not update this snapshot: construct
    a new instance to invalidate it. Tool configurations and unattached body
    poses remain fixed unless explicit synchronized paths are supplied.
    Use a context manager; close is explicit and idempotent.
    Calls on one instance serialize so closing cannot race an active query.
    """

    def __init__(self, robot_cell, robot_cell_state):
        if not hasattr(robot_cell, "compute_attach_objects_frames"):
            raise TypeError("robot_cell must be a COMPAS FAB 2 RobotCell")
        robot_cell.assert_cell_state_match(robot_cell_state)
        self._lock = RLock()
        self._closed = False
        self._registered_ids = []
        self._robot_cell = robot_cell.copy()
        self._robot_cell_state = robot_cell_state.copy()
        try:
            self._prepare()
        except BaseException:
            self.close()
            raise

    def _prepare(self):
        robot_cell = self._robot_cell
        robot_cell_state = self._robot_cell_state
        model = robot_cell.robot_model
        prefix = f"compas-fab-prepared:{uuid4().hex}"
        registered_ids = self._registered_ids
        link_records = _compas_fab_collision_links(robot_cell, prefix, registered_ids)
        records = {
            f"link:{record['name']}": {
                "key": f"link:{record['name']}",
                "kind": "link",
                "name": record["name"],
                "mesh_id": record["mesh_id"],
            }
            for record in link_records
        }

        visible_tools = {}
        for name in sorted(robot_cell.tool_models):
            state = robot_cell_state.tool_states[name]
            if state.is_hidden:
                continue
            mesh = _combined_model_collision_mesh(
                robot_cell.tool_models[name], state.configuration
            )
            mesh_id = f"{prefix}:tool:{name}"
            register_mesh_to_cache(mesh_id, mesh)
            registered_ids.append(mesh_id)
            key = f"tool:{name}"
            records[key] = {
                "key": key,
                "kind": "tool",
                "name": name,
                "mesh_id": mesh_id,
                "origin_radius": max(math.hypot(*mesh.vertex_coordinates(v)) for v in mesh.vertices()),
            }
            visible_tools[name] = state

        visible_bodies = {}
        for name in sorted(robot_cell.rigid_body_models):
            state = robot_cell_state.rigid_body_states[name]
            if state.is_hidden:
                continue
            mesh = _combined_rigid_body_collision_mesh(
                robot_cell.rigid_body_models[name]
            )
            mesh_id = f"{prefix}:body:{name}"
            register_mesh_to_cache(mesh_id, mesh)
            registered_ids.append(mesh_id)
            key = f"body:{name}"
            records[key] = {
                "key": key,
                "kind": "body",
                "name": name,
                "mesh_id": mesh_id,
                "origin_radius": max(math.hypot(*mesh.vertex_coordinates(v)) for v in mesh.vertices()),
            }
            visible_bodies[name] = state

        pair_specs = []
        adjacency = {
            frozenset((joint.parent.link, joint.child.link))
            for joint in model.joints
            if joint.parent is not None and joint.child is not None
        }
        disabled = _robot_semantic_disabled_collision_pairs(robot_cell)
        for index, first in enumerate(link_records):
            for second in link_records[index + 1:]:
                names = frozenset((first["name"], second["name"]))
                if names not in adjacency and names not in disabled:
                    pair_specs.append(
                        ("robot_self", f"link:{first['name']}", f"link:{second['name']}")
                    )

        for link in link_records:
            link_key = f"link:{link['name']}"
            for tool_name, tool_state in visible_tools.items():
                if link["name"] not in tool_state.touch_links:
                    pair_specs.append(("robot_tool", link_key, f"tool:{tool_name}"))
            for body_name, body_state in visible_bodies.items():
                if link["name"] not in body_state.touch_links:
                    pair_specs.append(("robot_body", link_key, f"body:{body_name}"))

        body_names = sorted(visible_bodies)
        for index, first_name in enumerate(body_names):
            first_state = visible_bodies[first_name]
            for second_name in body_names[index + 1:]:
                second_state = visible_bodies[second_name]
                first_attached = bool(
                    first_state.attached_to_link or first_state.attached_to_tool
                )
                second_attached = bool(
                    second_state.attached_to_link or second_state.attached_to_tool
                )
                if not (first_attached or second_attached):
                    continue
                if (
                    second_name in first_state.touch_bodies
                    or first_name in second_state.touch_bodies
                ):
                    continue
                pair_specs.append(
                    ("body_body", f"body:{first_name}", f"body:{second_name}")
                )

        for tool_name in sorted(visible_tools):
            for body_name, body_state in visible_bodies.items():
                if body_state.attached_to_tool == tool_name:
                    continue
                if tool_name in body_state.touch_bodies:
                    continue
                pair_specs.append(
                    ("tool_body", f"tool:{tool_name}", f"body:{body_name}")
                )

        pair_counts = {}
        for collision_type, _, _ in pair_specs:
            pair_counts[collision_type] = pair_counts.get(collision_type, 0) + 1


        self._link_records = link_records
        self._records = records
        self._pair_specs = pair_specs
        self._pair_counts = pair_counts
        self._visible_tools = visible_tools
        self._visible_bodies = visible_bodies
        self._motion_records = None
        self._dynamic_tool_records = {}

    def _motion_bound_records(self):
        """Bake fixed attachment placement into derivative-bound radii once.

        Entity motion is C * T_parent(q) * F with constant F. Its material
        points start within ||translation(F)|| + local_radius of the root
        reference origin. The endpoint surrogate still uses local_radius.
        Unattached tools and bodies have constant world poses and zero error.
        """
        if self._motion_records is not None:
            return self._motion_records
        cell = self._robot_cell
        state = self._robot_cell_state
        bound_records = [dict(record) for record in self._link_records]
        attachment_links = {}

        def tool_parent(name):
            group = state.tool_states[name].attached_to_group
            return cell.robot_semantics.get_end_effector_link_name(group) if group else None

        for name in self._visible_tools:
            attachment_links['tool:'+name] = tool_parent(name)
        for name,body in self._visible_bodies.items():
            parent = body.attached_to_link
            if body.attached_to_tool:
                parent = tool_parent(body.attached_to_tool)
                if parent is None:
                    raise ValueError('bounded attached workpiece requires a robot-attached tool')
            attachment_links['body:'+name] = parent

        entity_poses = self._pose_map(state.robot_configuration)
        parent_records = [dict(name=name,link=cell.robot_model.get_link_by_name(name))
                          for name in sorted(set(attachment_links.values())-{None})]
        parent_poses = _robot_link_pose_map(cell,state.robot_configuration,parent_records,state.robot_base_frame)
        for key,parent in attachment_links.items():
            local_radius = self._records[key]['origin_radius']
            if parent is None:
                radius = 0.0
                link = None
            else:
                offset = math.dist(entity_poses[key][:3],parent_poses[parent][:3])
                radius = offset+local_radius
                link = cell.robot_model.get_link_by_name(parent)
            if not math.isfinite(radius):
                raise ValueError('nonfinite attachment radius')
            bound_records.append(dict(key=key,name=key,link=link,origin_radius=radius,
                                      surrogate_radius=local_radius))
        self._motion_records = bound_records
        return bound_records

    @property
    def mesh_count(self):
        return len(self._registered_ids)

    def close(self):
        with self._lock:
            for mesh_id in self._registered_ids:
                unregister_mesh(mesh_id)
            self._registered_ids.clear()
            self._closed = True

    def __enter__(self):
        with self._lock:
            if self._closed:
                raise RuntimeError("PreparedRobotCell is closed")
            return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def sweep(self, trajectory, max_joint_step=math.radians(5.0),
              max_prismatic_step=0.01, max_subdivisions=128, parallel=True):
        """Reuse the prepared geometry for a collision trajectory query."""
        with self._lock:
            return self._run(trajectory, max_joint_step, max_prismatic_step,
                             max_subdivisions, parallel, None)

    def verify_clearance(self, trajectory, clearance, numerical_margin=1e-8,
                         time_tolerance=1e-5, max_evaluations=4096,
                         max_joint_step=math.radians(5.0),
                         max_prismatic_step=0.01, max_subdivisions=128,
                         parallel=True, solid=False, articulated_tolerance=None,
                         max_articulated_refinements=2, solid_rule="single_shell", scene_poses=None,
                         tool_trajectories=None):
        """Check surface clearance for interpolated link motion.

        Clear is conditional on the numerical margin bounding distance error.
        The numerical margin does not include physical error. An optional
        articulated_tolerance bounds joint-interpolation error for fixed-state
        tools, rigid attachments and static bodies as well as robot links.
        max_evaluations is per pair per subsegment.
        Unresolved articulated pairs may be bisected locally up to
        max_articulated_refinements (0 disables refinement, maximum 8).
        scene_poses maps visible unattached body/tool keys to one world pose per
        trajectory point. Translation is linear and rotation shortest-arc;
        synchronization follows the same segment parameter as the joint path.
        tool_trajectories maps tool names to complete JointTrajectory objects
        with the robot path's point times. It requires articulated_tolerance.
        """
        options = dict(clearance=clearance, numerical_margin=numerical_margin,
                       time_tolerance=time_tolerance, max_evaluations=max_evaluations,
                       solid=solid, solid_rule=solid_rule)
        # Validate options even when all pairs are excluded.
        verify_clearance_cached_batch_poses([], parallel=parallel, **options)
        if (isinstance(max_articulated_refinements, bool)
                or not isinstance(max_articulated_refinements, int)
                or not 0 <= max_articulated_refinements <= 8):
            raise ValueError('max_articulated_refinements must be an integer in [0, 8]')
        with self._lock:
            result = self._run(trajectory, max_joint_step, max_prismatic_step,
                               max_subdivisions, parallel, options, articulated_tolerance,
                               max_articulated_refinements, scene_poses, tool_trajectories)
            result.update(options)
            result['max_articulated_refinements'] = max_articulated_refinements
            result['moving_scene_entities'] = sorted(scene_poses or {})
            result['scene_motion_model'] = 'synchronized_linear_translation_shortest_arc_rotation'
            result['moving_tool_joint_models'] = sorted(tool_trajectories or {})
            if articulated_tolerance is not None:
                entities = {record.get('key','link:'+record['name'])
                            for record in self._motion_bound_records()}
                for name in tool_trajectories or {}:
                    entities.discard('tool:'+name)
                    entities.update(f'tool:{name}/link:{r["name"]}'
                                    for r in self._dynamic_tool_records[name])
                result['bounded_entities'] = sorted(entities)
            result['numerical_error_bound_proven'] = False
            result['distance_semantics'] = (
                'even_odd_shell_solid_clearance_under_contract' if solid_rule == 'even_odd'
                else 'single_shell_solid_clearance_under_contract' if solid
                else 'surface_distance_not_solid_containment')
            return result

    def _pose_map(self, configuration, tool_configurations=None, scene_overrides=None):
        robot_cell = self._robot_cell
        robot_cell_state = self._robot_cell_state
        link_records = self._link_records
        visible_tools = self._visible_tools
        visible_bodies = self._visible_bodies
        state = robot_cell_state.copy()
        state.robot_configuration = configuration
        for name,config in (tool_configurations or {}).items():
            state.tool_states[name].configuration = config
        if scene_overrides:
            from compas.geometry import Frame, Quaternion
            for key,pose in scene_overrides.items():
                kind,name = key.split(':',1)
                frame = Frame.from_quaternion(Quaternion(pose[6],*pose[3:6]),point=pose[:3])
                (state.tool_states if kind=='tool' else state.rigid_body_states)[name].frame = frame
        state = robot_cell.compute_attach_objects_frames(state)
        poses = _robot_link_pose_map(
            robot_cell, configuration, link_records, state.robot_base_frame
        )
        result = {f"link:{name}": pose for name, pose in poses.items()}
        for name in visible_tools:
            frame = state.tool_states[name].frame
            if frame is None:
                raise ValueError(f"visible tool '{name}' has no resolvable frame")
            result[f"tool:{name}"] = pose_from_frame(frame)
            if name in (tool_configurations or {}):
                poses = _robot_link_pose_map(robot_cell.tool_models[name],tool_configurations[name],
                                            self._dynamic_tool_records[name],frame)
                result.update({f'tool:{name}/link:{link}':pose for link,pose in poses.items()})
        for name in visible_bodies:
            frame = state.rigid_body_states[name].frame
            if frame is None:
                raise ValueError(f"visible rigid body '{name}' has no resolvable frame")
            result[f"body:{name}"] = pose_from_frame(frame)
        return result


    def _validate_scene_paths(self, scene_poses, count):
        if scene_poses is None:
            return {}
        if not isinstance(scene_poses, dict):
            raise TypeError('scene_poses must be a mapping of entity keys to poses')
        paths = {}
        for key, values in scene_poses.items():
            if key not in self._records or self._records[key]['kind'] not in ('body','tool'):
                raise ValueError('scene_poses requires visible body/tool entity keys')
            kind, name = key.split(':',1)
            state = (self._visible_bodies if kind == 'body' else self._visible_tools)[name]
            attached = ((state.attached_to_link or state.attached_to_tool) if kind == 'body'
                        else state.attached_to_group)
            if attached:
                raise ValueError('scene_poses cannot override an attached entity')
            if kind == 'tool' and any(s.attached_to_tool == name for s in self._visible_bodies.values()):
                raise ValueError('moving free tools with attached bodies require an explicit coupled trajectory')
            path = [_coerce_pose(value) for value in values]
            if len(path) != count:
                raise ValueError('scene_poses needs one pose per trajectory point')
            for pose in path:
                if not all(math.isfinite(v) for v in pose):
                    raise ValueError('scene poses must be finite')
                norm = math.hypot(*pose[3:])
                if norm == 0 or not math.isfinite(norm):
                    raise ValueError('scene pose quaternion must be finite and nonzero')
                pose[3:] = [v/norm for v in pose[3:]]
            paths[key] = path
        return paths

    def _validate_tool_paths(self, trajectories, robot_trajectory, tolerance):
        if trajectories is None:
            return {}
        if not isinstance(trajectories,dict):
            raise TypeError('tool_trajectories must map visible tool names to JointTrajectory objects')
        if trajectories and tolerance is None:
            raise ValueError('moving tool joints require articulated_tolerance')
        paths = {}
        robot_times = _trajectory_times(robot_trajectory)
        for name,path in trajectories.items():
            if name not in self._visible_tools:
                raise ValueError('tool trajectory requires a visible tool')
            model = self._robot_cell.tool_models[name]
            _require_valid_compas_fab_trajectory(model,path)
            if _trajectory_times(path) != robot_times:
                raise ValueError('tool and robot trajectories must have identical point times and counts')
            configs = [_configuration_from_trajectory_point(p,path.joint_names) for p in path.points]
            required = set(model.get_configurable_joint_names())
            if any(not required.issubset(c.joint_names) for c in configs):
                raise ValueError('tool trajectories require complete configurable joint values')
            if name not in self._dynamic_tool_records:
                owned = []
                try:
                    records = _compas_fab_collision_links(model,f'forge-tool:{uuid4().hex}',owned)
                except BaseException:
                    for mesh_id in owned:
                        unregister_mesh(mesh_id)
                    raise
                self._registered_ids.extend(owned)
                self._dynamic_tool_records[name] = records
                for record in records:
                    key = f'tool:{name}/link:{record["name"]}'
                    self._records[key] = dict(record,key=key,kind='tool_link')
            paths[name] = configs
        return paths

    def _refine_pair(self, query, initial_result, start, end, allowance, options, depth, pose_sampler=None):
        """Refine one bounded joint interval, preserving unknown and actual witnesses.

        Endpoint FK is memoized within this pair. All reported times remain in
        the original subsegment's normalized coordinates. Work is depth bounded.
        """
        pose_cache = {}
        counts = dict(distance_evaluations=0, pair_queries=0, fk_evaluations=0, broadphase_rejections=0)

        def poses(t):
            if t not in pose_cache:
                pose_cache[t] = (pose_sampler(t) if pose_sampler is not None else
                                 self._pose_map(_interpolate_configuration(start, end, t)))
                counts['fk_evaluations'] += 1
            return pose_cache[t]

        def check(lo, hi, offset):
            a, b = poses(lo), poses(hi)
            q = (query[4], a[query[1]], b[query[1]], query[7], a[query[2]], b[query[2]])
            r = verify_clearance_cached_batch_poses([q], parallel=False,
                clearance_offsets=[offset], **options)[0]
            counts['distance_evaluations'] += r['distance_evaluations']
            counts['pair_queries'] += 1
            counts['broadphase_rejections'] += int(r.get('broadphase_rejected',False))
            return r

        def visit(lo, hi, remaining, result=None):
            result = result if result is not None else check(lo, hi, allowance*(hi-lo)**2)
            if result['status'] == 'clear':
                return result
            if result['status'] == 'violation':
                t = lo+(hi-lo)*result['witness_time']
                checked = check(t, t, 0.0)
                if checked['status'] == 'violation':
                    return dict(checked, witness_time=t)
                result = dict(result, status='unknown',
                    reason='inflated_surrogate_violation_not_confirmed_on_articulated_path',
                    witness_time=None, witness_distance=None)
            result = dict(result, unresolved_interval=(lo, hi))
            if remaining == 0:
                return result
            mid = (lo+hi)*.5
            left = visit(lo, mid, remaining-1)
            if left['status'] == 'violation':
                return left
            right = visit(mid, hi, remaining-1)
            if right['status'] == 'violation':
                return right
            if left['status'] != 'clear':
                return left
            return right

        return visit(0.0, 1.0, depth, initial_result), counts

    def _run(self, trajectory, max_joint_step, max_prismatic_step,
             max_subdivisions, parallel, clearance_options, articulated_tolerance=None,
             max_articulated_refinements=0, scene_poses=None, tool_trajectories=None):
        if self._closed:
            raise RuntimeError("PreparedRobotCell is closed")
        robot_cell = self._robot_cell
        robot_cell_state = self._robot_cell_state
        if articulated_tolerance is not None:
            if not math.isfinite(articulated_tolerance) or articulated_tolerance <= 0:
                raise ValueError('articulated_tolerance must be finite and positive')
            from .motion_bounds import link_chord_coefficients
        points = list(getattr(trajectory, "points", []))
        if len(points) < 2:
            raise ValueError("trajectory must contain at least two points")
        scene_paths = self._validate_scene_paths(scene_poses, len(points))
        tool_paths = self._validate_tool_paths(tool_trajectories,trajectory,articulated_tolerance)
        validation = _require_valid_compas_fab_trajectory(robot_cell, trajectory)
        trajectory_times = _trajectory_times(trajectory)
        if not math.isfinite(max_joint_step) or max_joint_step <= 0.0:
            raise ValueError("max_joint_step must be finite and greater than zero")
        if not math.isfinite(max_prismatic_step) or max_prismatic_step <= 0.0:
            raise ValueError("max_prismatic_step must be finite and greater than zero")
        if isinstance(max_subdivisions, bool) or not isinstance(max_subdivisions, int) or max_subdivisions < 1:
            raise ValueError("max_subdivisions must be a positive integer")
        if not isinstance(parallel, bool):
            raise TypeError("parallel must be a bool")

        model = robot_cell.robot_model
        configurations = [
            _configuration_from_trajectory_point(
                point, trajectory.joint_names or model.get_configurable_joint_names()
            )
            for point in points
        ]
        # A partial trajectory overrides only named joints. Keep the remaining
        # robot state, rather than allowing model FK to silently default it.
        configurations = [
            robot_cell_state.robot_configuration.merged(config)
            for config in configurations
        ]
        for configuration in configurations:
            _validate_full_configuration(robot_cell.robot_model,configuration)

        link_records = self._link_records
        records = self._records
        pair_specs = list(self._pair_specs)
        # The FAB snapshot policy excludes world/world bodies. A moving world
        # body requires these pairs, and moving tools require tool/tool pairs.
        existing = {frozenset((a,b)) for _,a,b in pair_specs}
        for key in sorted(scene_paths):
            kind,name = key.split(':',1)
            candidates = self._visible_bodies if kind == 'body' else self._visible_tools
            for other_name in sorted(candidates):
                other = kind+':'+other_name
                pair = frozenset((key,other))
                if key == other or pair in existing:
                    continue
                if kind == 'body' and (other_name in candidates[name].touch_bodies
                                      or name in candidates[other_name].touch_bodies):
                    continue
                pair_specs.append((kind+'_'+kind,*sorted((key,other))))
                existing.add(pair)
        pair_specs.sort()
        if tool_paths:
            # Dynamic tools have retained per-link meshes, never a frozen union.
            for name in sorted(tool_paths):
                for other in sorted(self._visible_tools):
                    pair = frozenset((f'tool:{name}',f'tool:{other}'))
                    if name != other and pair not in existing:
                        pair_specs.append(('tool_tool',*sorted(pair)))
                        existing.add(pair)
            replacements = {f'tool:{name}':[f'tool:{name}/link:{r["name"]}'
                            for r in self._dynamic_tool_records[name]] for name in tool_paths}
            pair_specs = [(kind,a,b) for kind,x,y in pair_specs
                          for a in replacements.get(x,[x]) for b in replacements.get(y,[y])]
            for name in tool_paths:
                model = self._robot_cell.tool_models[name]
                adjacency = {frozenset((j.parent.link,j.child.link)) for j in model.joints}
                tool_records = self._dynamic_tool_records[name]
                for i,a in enumerate(tool_records):
                    for b in tool_records[i+1:]:
                        if frozenset((a['name'],b['name'])) not in adjacency:
                            pair_specs.append(('tool_self',f'tool:{name}/link:{a["name"]}',
                                               f'tool:{name}/link:{b["name"]}'))
            pair_specs.sort()
        pair_counts = {}
        for kind,_,_ in pair_specs:
            pair_counts[kind] = pair_counts.get(kind,0)+1
        distance_evaluations = 0
        evaluated_subsegments = 0
        evaluated_pair_queries = 0
        broadphase_rejections = 0
        fk_evaluations = 0
        refinement_pair_queries = 0
        original_segment_count = len(configurations) - 1
        maximum_motion_error = 0.0
        for segment_index, (start, end) in enumerate(
            zip(configurations, configurations[1:])
        ):
            ratios = []
            for value_a, value_b, joint_type in zip(
                start.joint_values, end.joint_values, start.joint_types
            ):
                delta = abs(value_b - value_a)
                if joint_type in (Joint.REVOLUTE, Joint.CONTINUOUS):
                    ratios.append(delta / max_joint_step)
                elif joint_type == Joint.PRISMATIC:
                    ratios.append(delta / max_prismatic_step)
            subdivisions = max(1, math.ceil(max(ratios, default=0.0)))
            motion_error = 0.0
            entity_errors = {}
            if articulated_tolerance is not None:
                coefficients = link_chord_coefficients(robot_cell.robot_model, self._motion_bound_records(), start, end)
                if tool_paths:
                    from .motion_bounds import tool_link_chord_coefficient
                    from compas.geometry import Quaternion
                    base_records = {r.get('key','link:'+r['name']):r for r in self._motion_bound_records()}
                    for name,path in tool_paths.items():
                        root_key = 'tool:'+name
                        root_record = base_records[root_key]
                        angle = 0.0
                        if root_key in scene_paths:
                            p0,p1 = scene_paths[root_key][segment_index:segment_index+2]
                            q0,q1 = Quaternion(p0[6],*p0[3:6]),Quaternion(p1[6],*p1[3:6])
                            angle = 2*math.acos(min(1.0,abs(q0.dot(q1))))
                        del coefficients[root_key]
                        for record in self._dynamic_tool_records[name]:
                            key = f'tool:{name}/link:{record["name"]}'
                            coefficients[key] = tool_link_chord_coefficient(
                                robot_cell.tool_models[name],record,path[segment_index],path[segment_index+1],
                                robot_cell.robot_model,root_record['link'],start,end,
                                root_record['origin_radius']-root_record['surrogate_radius'],angle)
                coefficient = max((v[0] for v in coefficients.values()), default=0.0)
                angular = max((v[1] for v in coefficients.values()), default=0.0)
                subdivisions = max(subdivisions, math.ceil(math.sqrt(coefficient/articulated_tolerance)),
                                   math.ceil(angular/math.pi))
                motion_error = coefficient/(subdivisions*subdivisions)
                entity_errors = {key: value[0]/(subdivisions*subdivisions)
                                 for key, value in coefficients.items()}
                maximum_motion_error = max(maximum_motion_error,motion_error)
            if subdivisions > max_subdivisions:
                raise ValueError(
                    f"segment {segment_index} requires {subdivisions} subdivisions, "
                    f"above max_subdivisions={max_subdivisions}"
                )

            sampled = [
                _interpolate_configuration(start, end, i / subdivisions)
                for i in range(subdivisions + 1)
            ]
            def sample_pose(t):
                tool_configs = {name:_interpolate_configuration(path[segment_index],path[segment_index+1],t)
                                for name,path in tool_paths.items()}
                overrides = {key:_interpolate_scene_pose(path[segment_index],path[segment_index+1],t)
                             for key,path in scene_paths.items()}
                return self._pose_map(_interpolate_configuration(start,end,t),tool_configs,overrides)

            pose_maps = [sample_pose(i/subdivisions) for i in range(subdivisions+1)]
            fk_evaluations += len(pose_maps)

            for subsegment_index in range(subdivisions):
                evaluated_subsegments += 1
                start_poses = pose_maps[subsegment_index]
                end_poses = pose_maps[subsegment_index + 1]
                queries = []
                for collision_type, key_a, key_b in pair_specs:
                    record_a = records[key_a]
                    record_b = records[key_b]
                    queries.append(
                        (
                            collision_type,
                            key_a,
                            key_b,
                            key_b,
                            record_a["mesh_id"],
                            start_poses[key_a],
                            end_poses[key_a],
                            record_b["mesh_id"],
                            start_poses[key_b],
                            end_poses[key_b],
                        )
                    )
                evaluated_pair_queries += len(queries)
                if clearance_options is not None:
                    query_options = dict(clearance_options)
                    # Distance is 1-Lipschitz in each body's displacement.
                    # Use E_a + E_b, not twice the largest error in the cell.
                    if articulated_tolerance is not None:
                        query_options['clearance_offsets'] = [
                            entity_errors[q[1]] + entity_errors[q[2]] for q in queries
                        ]
                    clearance_results = verify_clearance_cached_batch_poses(
                        [query[4:] for query in queries], parallel=parallel,
                        **query_options
                    )
                    distance_evaluations += sum(r["distance_evaluations"] for r in clearance_results)
                    broadphase_rejections += sum(bool(r.get('broadphase_rejected')) for r in clearance_results)
                    if articulated_tolerance is not None:
                        for i, (query, result) in enumerate(zip(queries, clearance_results)):
                            if result['status'] == 'clear':
                                continue
                            allowance = entity_errors[query[1]] + entity_errors[query[2]]
                            refined, counts = self._refine_pair(
                                query, result, sampled[subsegment_index], sampled[subsegment_index+1],
                                allowance, clearance_options, max_articulated_refinements,
                                (lambda t: sample_pose((subsegment_index+t)/subdivisions)) if (scene_paths or tool_paths) else None)
                            clearance_results[i] = refined
                            distance_evaluations += counts['distance_evaluations']
                            refinement_pair_queries += counts['pair_queries']
                            evaluated_pair_queries += counts['pair_queries']
                            fk_evaluations += counts['fk_evaluations']
                            broadphase_rejections += counts['broadphase_rejections']
                    outcomes = [(q, r) for q, r in zip(queries, clearance_results) if r["status"] != "clear"]
                    if outcomes:
                        # Stop at the first unresolved/violating subsegment. A witness
                        # is not an earliest threshold-crossing time.
                        query, result = min(outcomes, key=lambda qr: (
                            0 if qr[1]["status"] == "violation" else 1, qr[0][:3]
                        ))
                        witness = result["witness_time"]
                        fraction = ((subsegment_index + witness) / subdivisions
                                    if witness is not None else None)
                        return {
                            "status": result["status"], "entity_a": query[1],
                            "entity_b": query[2], "collision_type": query[0],
                            "segment_index": segment_index,
                            "subsegment_index": subsegment_index,
                            "subdivisions_in_segment": subdivisions,
                            "segment_result": result,
                            "distance_evaluations": distance_evaluations,
                            "refinement_pair_queries": refinement_pair_queries,
                            "broadphase_rejections": broadphase_rejections,
                            "fk_evaluations": fk_evaluations,
                            "evaluated_pair_queries": evaluated_pair_queries,
                            "pair_counts": dict(pair_counts),
                            "motion_model": ('bounded_linear_joint_path' if articulated_tolerance is not None
                                             else 'piecewise_rigid_between_joint_samples'),
                            "maximum_link_deviation_bound": (maximum_motion_error if articulated_tolerance is not None else None),
                            "articulated_tolerance": articulated_tolerance,
                            "pair_motion_allowance": entity_errors.get(query[1], 0.0) + entity_errors.get(query[2], 0.0),
                            "motion_allowance_policy": "sum_of_pair_entity_bounds",
                            "witness_time_from_start": _collision_timing(
                                trajectory_times, validation["timing_supplied"],
                                segment_index, fraction
                            )["collision_time_from_start"],
                        }
                    continue
                native_results = check_swept_collision_cached_batch_poses(
                    [query[4:] for query in queries], parallel=parallel
                )
                candidates = []
                for query, result in zip(queries, native_results):
                    collision_type, key_a, key_b, _ = query[:4]
                    if result["method"] in ("swept_sphere_aabb_rejected", "swept_endpoint_aabb_rejected"):
                        broadphase_rejections += 1
                    if result["has_collision"]:
                        candidates.append(
                            (result["time_of_impact"], collision_type, key_a, key_b, result)
                        )
                if candidates:
                    candidates.sort(key=lambda item: item[:4])
                    local_toi, collision_type, key_a, key_b, result = candidates[0]
                    fraction_in_segment = (subsegment_index + local_toi) / subdivisions
                    return {
                        "has_collision": True,
                        "collision_type": collision_type,
                        "entity_a": key_a,
                        "entity_b": key_b,
                        "segment_index": segment_index,
                        "subsegment_index": subsegment_index,
                        "subdivisions_in_segment": subdivisions,
                        "segment_fraction": fraction_in_segment,
                        "trajectory_fraction": (
                            segment_index + fraction_in_segment
                        ) / original_segment_count,
                        "segment_result": result,
                        "pair_counts": dict(pair_counts),
                        "evaluated_subsegments": evaluated_subsegments,
                        "evaluated_pair_queries": evaluated_pair_queries,
                        "broadphase_rejections": broadphase_rejections,
                        "fk_evaluations": fk_evaluations,
                        "execution_mode": (
                            "native_rayon_batch" if parallel else "native_serial_batch"
                        ),
                        **_collision_timing(
                            trajectory_times,
                            validation["timing_supplied"],
                            segment_index,
                            fraction_in_segment,
                        ),
                    }

        if clearance_options is not None:
            return {
                "status": "clear", "entity_a": None, "entity_b": None,
                "segment_result": None,
                "distance_evaluations": distance_evaluations,
                "refinement_pair_queries": refinement_pair_queries,
                "broadphase_rejections": broadphase_rejections,
                "fk_evaluations": fk_evaluations,
                "evaluated_pair_queries": evaluated_pair_queries,
                "pair_counts": dict(pair_counts),
                "motion_model": ('bounded_linear_joint_path' if articulated_tolerance is not None
                                 else 'piecewise_rigid_between_joint_samples'),
                "maximum_link_deviation_bound": (maximum_motion_error if articulated_tolerance is not None else None),
                "articulated_tolerance": articulated_tolerance,
                "motion_allowance_policy": "sum_of_pair_entity_bounds",
                "witness_time_from_start": None,
            }
        return {
            "has_collision": False,
            "collision_type": None,
            "entity_a": None,
            "entity_b": None,
            "segment_index": None,
            "subsegment_index": None,
            "subdivisions_in_segment": None,
            "segment_fraction": None,
            "trajectory_fraction": None,
            "segment_result": None,
            "pair_counts": dict(pair_counts),
            "evaluated_subsegments": evaluated_subsegments,
            "evaluated_pair_queries": evaluated_pair_queries,
            "broadphase_rejections": broadphase_rejections,
            "fk_evaluations": fk_evaluations,
            "execution_mode": (
                "native_rayon_batch" if parallel else "native_serial_batch"
            ),
            **_collision_timing(
                trajectory_times, validation["timing_supplied"], None, None
            ),
        }
