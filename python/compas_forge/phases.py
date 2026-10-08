"""Geometric preflight of explicit attachment phases, never grasp-force validation."""
from contextlib import ExitStack
import math

from .cell import PreparedRobotCell, _validate_full_configuration
from . import _require_valid_compas_fab_trajectory, _configuration_from_trajectory_point


def verify_clearance_phases(robot_cell, phases, clearance, *,
                            continuity_tolerance=1e-9, **options):
    """Verify one geometry model through piecewise-constant attachment states.

    Each phase is a dict with state, trajectory and optional scene_poses and
    tool_trajectories. Times are local to each phase. Every transition must have
    continuous joint values and world geometry poses. Reparenting and explicit
    touch exclusions may change at a boundary. No interpolation across a grasp
    event, physical grasp stability, execution timing or dynamics is inferred.
    All boundary validation happens before any clear result can be returned.
    """
    if not math.isfinite(continuity_tolerance) or continuity_tolerance < 0:
        raise ValueError('continuity_tolerance must be finite and nonnegative')
    phases = list(phases)
    if not phases:
        raise ValueError('at least one phase is required')
    if {'scene_poses','tool_trajectories'} & options.keys():
        raise ValueError('motion paths belong to individual phases')

    def endpoint(prepared, path, tool_paths, scene_paths, index):
        partial = _configuration_from_trajectory_point(path.points[index],path.joint_names)
        q = prepared._robot_cell_state.robot_configuration.merged(partial)
        _validate_full_configuration(prepared._robot_cell.robot_model,q)
        tool_configs = {name:configs[index] for name,configs in tool_paths.items()}
        # Include every tool's links even in a static phase so a changed tool
        # configuration cannot evade boundary continuity through its root frame.
        for name in prepared._visible_tools:
            tool_configs.setdefault(name,prepared._robot_cell_state.tool_states[name].configuration)
            if name not in prepared._dynamic_tool_records:
                from . import _compas_fab_collision_links
                from uuid import uuid4
                prepared._dynamic_tool_records[name] = _compas_fab_collision_links(
                    prepared._robot_cell.tool_models[name],f'phase:{uuid4().hex}',prepared._registered_ids)
        poses = prepared._pose_map(q,tool_configs,{k:v[index] for k,v in scene_paths.items()})
        joints = {'robot:'+k:v for k,v in zip(q.joint_names,q.joint_values)}
        for name,config in tool_configs.items():
            if config is not None:
                joints.update({f'tool:{name}:{k}':v for k,v in zip(config.joint_names,config.joint_values)})
        return poses,joints

    with ExitStack() as stack:
        prepared_phases = []
        previous = None
        for index,phase in enumerate(phases):
            if not isinstance(phase,dict) or set(phase)-{'state','trajectory','scene_poses','tool_trajectories'}:
                raise ValueError('phase requires state, trajectory and only supported motion fields')
            if not {'state','trajectory'} <= phase.keys():
                raise ValueError('phase requires state and trajectory')
            prepared = stack.enter_context(PreparedRobotCell(robot_cell,phase['state']))
            path = phase['trajectory']
            _require_valid_compas_fab_trajectory(robot_cell,path)
            if len(path.points) < 2:
                raise ValueError('each phase needs at least two trajectory points')
            tools = prepared._validate_tool_paths(phase.get('tool_trajectories'),path,options.get('articulated_tolerance'))
            scene = prepared._validate_scene_paths(phase.get('scene_poses'),len(path.points))
            first = endpoint(prepared,path,tools,scene,0)
            if previous is not None:
                old_poses,old_joints = previous
                poses,joints = first
                if old_poses.keys() != poses.keys() or old_joints.keys() != joints.keys():
                    raise ValueError(f'phase {index}: visible geometry/joint set changed')
                if any(abs(joints[k]-old_joints[k]) > continuity_tolerance for k in joints):
                    raise ValueError(f'phase {index}: discontinuous joint values')
                for key,p in poses.items():
                    old = old_poses[key]
                    # q and -q represent the same rotation. Use quaternion chord
                    # distance, avoiding acos amplification near identical poses.
                    rotation = min(math.dist(p[3:],old[3:]),math.dist(p[3:],[-v for v in old[3:]]))
                    if math.dist(p[:3],old[:3]) > continuity_tolerance or rotation > continuity_tolerance:
                        raise ValueError(f'phase {index}: discontinuous world pose for {key}')
            previous = endpoint(prepared,path,tools,scene,-1)
            prepared_phases.append((prepared,phase))
        results = [prepared.verify_clearance(phase['trajectory'],clearance,
                    scene_poses=phase.get('scene_poses'),tool_trajectories=phase.get('tool_trajectories'),**options)
                   for prepared,phase in prepared_phases]
    status = 'violation' if any(r['status']=='violation' for r in results) else (
        'unknown' if any(r['status']=='unknown' for r in results) else 'clear')
    return dict(status=status,phases=results,validated_transitions=len(phases)-1,
                continuity_tolerance=continuity_tolerance,
                grasp_stability_verified=False,robot_commands_sent=False,
                motion_contract='continuous_geometry_piecewise_fixed_attachments')
