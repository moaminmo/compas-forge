"""Numerical counterexample search supporting, not replacing, the bound derivation."""
import math
import pytest
from compas.geometry import Rotation
from compas_fab.robots import RobotCellLibrary, JointTrajectory, JointTrajectoryPoint
from compas_forge import prepare_compas_fab_cell, _interpolate_configuration
from compas_forge.motion_bounds import link_chord_coefficients


def trajectory(state, delta):
    config = state.robot_configuration
    a = JointTrajectoryPoint(config.joint_values,config.joint_types,joint_names=config.joint_names)
    b = JointTrajectoryPoint([v+delta for v in config.joint_values],config.joint_types,joint_names=config.joint_names)
    return JointTrajectory([a,b],joint_names=config.joint_names)


@pytest.mark.parametrize('factory',['ur5','ur10e','ur5_gripper_one_beam','ur10e_gripper_one_beam'])
def test_articulated_bounds_cover_sampled_material_point_deviation(factory):
    from compas.geometry import Frame
    cell,state = getattr(RobotCellLibrary,factory)()
    state.robot_base_frame = Frame([2,-1,.8],[0,1,0],[-1,0,0])
    start = state.robot_configuration
    end = start.copy()
    end.joint_values = [v+.4*(-1 if i%2 else 1) for i,v in enumerate(start.joint_values)]
    with prepare_compas_fab_cell(cell,state) as prepared:
        records = prepared._motion_bound_records()
        coefficients = link_chord_coefficients(cell.robot_model,records,start,end)
        for n in [1,4,16]:
            q1 = _interpolate_configuration(start,end,1/n)
            p0,p1 = prepared._pose_map(start),prepared._pose_map(q1)
            for t in [.1,.3,.5,.7,.9]:
                actual = prepared._pose_map(_interpolate_configuration(start,q1,t))
                for record in records:
                    key = record.get('key','link:'+record['name'])
                    # Use COMPAS quaternion slerp via rotation axis/angle of relative rotation.
                    from compas.geometry import Quaternion
                    r0 = Rotation.from_quaternion(Quaternion(p0[key][6],*p0[key][3:6]))
                    r1 = Rotation.from_quaternion(Quaternion(p1[key][6],*p1[key][3:6]))
                    axis,angle = (r1*r0.inverted()).axis_and_angle
                    rs = Rotation.from_axis_and_angle(axis,angle*t)*r0
                    ra = Rotation.from_quaternion(Quaternion(actual[key][6],*actual[key][3:6]))
                    radius = record.get('surrogate_radius',record['origin_radius'])
                    for v in [[radius,0,0],[0,0,-radius]]:
                        from compas.geometry import Vector
                        vs,va = Vector(*v).transformed(rs),Vector(*v).transformed(ra)
                        error = math.sqrt(sum((va[i]+actual[key][i]-vs[i]-(p0[key][i]*(1-t)+p1[key][i]*t))**2 for i in range(3)))
                        assert error <= coefficients[key][0]/n**2+1e-10


def test_robot_only_articulated_clearance_exposes_bound():
    cell,state = RobotCellLibrary.ur5()
    with prepare_compas_fab_cell(cell,state) as prepared:
        result = prepared.verify_clearance(trajectory(state,.01),.001,
            articulated_tolerance=.002,max_evaluations=32,parallel=False)
        assert result['motion_model'] == 'bounded_linear_joint_path'
        assert result['maximum_link_deviation_bound'] <= .002
        assert result['numerical_error_bound_proven'] is False


def test_articulated_cell_covers_gripper_beam_and_static_floor():
    cell,state = RobotCellLibrary.ur5_gripper_one_beam()
    with prepare_compas_fab_cell(cell,state) as prepared:
        result = prepared.verify_clearance(trajectory(state,0),.001,articulated_tolerance=.001,parallel=False)
        assert result['status'] == 'violation'
        assert {'tool:gripper','body:beam','body:floor'} <= set(result['bounded_entities'])
        assert {result['entity_a'],result['entity_b']} == {'body:beam','body:floor'}
        assert result['maximum_link_deviation_bound'] == 0.0


def test_moving_gripper_workpiece_path_is_checked_not_only_initial_pose():
    cell,state = RobotCellLibrary.ur5_gripper_one_beam()
    config = state.robot_configuration
    start = [0,-1.57,1.57,-1.57,-1.57,0]
    end = [.1,-1.57,1.57,-1.57,-1.57,0]
    path = JointTrajectory([JointTrajectoryPoint(v,config.joint_types,joint_names=config.joint_names)
                            for v in [start,end]],joint_names=config.joint_names)
    with prepare_compas_fab_cell(cell,state) as prepared:
        result = prepared.verify_clearance(path,.001,articulated_tolerance=.001,parallel=False)
    assert result['status'] == 'clear'
    assert result['evaluated_pair_queries'] > sum(result['pair_counts'].values())
    assert 0 < result['maximum_link_deviation_bound'] <= .001
    assert 'body:beam' in result['bounded_entities']


def test_workpiece_on_unattached_tool_is_rejected_in_bounded_mode():
    cell,state = RobotCellLibrary.ur5_gripper_one_beam()
    state.tool_states['gripper'].attached_to_group = None
    with prepare_compas_fab_cell(cell,state) as prepared:
        with pytest.raises(ValueError,match='robot-attached tool'):
            prepared.verify_clearance(trajectory(state,0),.001,articulated_tolerance=.001)


def test_direct_link_attachment_and_static_objects_have_explicit_bounds():
    from compas.geometry import Frame
    cell,state = RobotCellLibrary.ur5_gripper_one_beam()
    state.set_rigid_body_attached_to_link('beam','wrist_3_link',Frame([.3,.1,.2],[1,0,0],[0,1,0]))
    start = state.robot_configuration
    end = start.copy()
    end.joint_values = [v+.1 for v in start.joint_values]
    with prepare_compas_fab_cell(cell,state) as prepared:
        records = prepared._motion_bound_records()
        coefficients = link_chord_coefficients(cell.robot_model,records,start,end)
        assert coefficients['body:beam'][0] > 0
        assert coefficients['body:floor'] == (0,0)
        # Fixed attachment offset recovered at another configuration is invariant.
        beam = next(r for r in records if r.get('key')=='body:beam')
        poses = prepared._pose_map(end)
        parent = poses['link:wrist_3_link']
        expected = math.dist(poses['body:beam'][:3],parent[:3])+beam['surrogate_radius']
        assert beam['origin_radius'] == pytest.approx(expected,abs=1e-12)


@pytest.mark.parametrize('confirmed',[False,True])
def test_surrogate_witness_requires_actual_fk_confirmation(monkeypatch,confirmed):
    import compas_forge.cell as adapter
    cell,state = RobotCellLibrary.ur5()
    checks = []
    def oracle(queries,**kwargs):
        if not queries:
            return []
        checks.append(queries)
        status = 'violation' if len(queries)>1 or confirmed else 'clear'
        return [dict(status=status,witness_time=.5 if status=='violation' else None,
                     witness_distance=0.0 if status=='violation' else None,
                     distance_evaluations=1,reason='test_oracle') for q in queries]
    with prepare_compas_fab_cell(cell,state) as prepared:
        monkeypatch.setattr(adapter,'verify_clearance_cached_batch_poses',oracle)
        result = prepared.verify_clearance(trajectory(state,.01),.001,articulated_tolerance=.002,
                                          max_articulated_refinements=0)
        assert len(checks)>=2 and len(checks[1])==1
        assert checks[1][0][1] == checks[1][0][2]  # actual-pose static verification
        assert result['status'] == ('violation' if confirmed else 'unknown')
        if not confirmed:
            assert result['segment_result']['witness_time'] is None
            assert 'not_confirmed' in result['segment_result']['reason']


def test_rotating_prismatic_chain_derivative_bound_against_analytical_path():
    from types import SimpleNamespace as NS
    from compas_robots import Configuration
    from compas_robots.model import Joint
    rotation = NS(name='r',type=Joint.CONTINUOUS,mimic=None,
                  current_axis=NS(vector=[0,0,1]),current_origin=NS(point=[0,0,0]),
                  parent=NS(link='root'))
    slide = NS(name='p',type=Joint.PRISMATIC,mimic=None,
               current_axis=NS(vector=[1,0,0]),limit=NS(lower=0,upper=3),
               parent=NS(link='rotated'))
    model = NS(get_link_by_name=lambda name: NS(parent_joint=rotation if name=='rotated' else None))
    records = [dict(name='tip',origin_radius=1.0,link=NS(parent_joint=slide))]
    start = Configuration([0,0],[Joint.CONTINUOUS,Joint.PRISMATIC],['r','p'])
    end = Configuration([1,2],[Joint.CONTINUOUS,Joint.PRISMATIC],['r','p'])
    coefficient,_ = link_chord_coefficients(model,records,start,end)['link:tip']
    assert coefficient == pytest.approx(1.0)
    for n in [1,2,8]:
        # Actual material point (1+2t)*(cos(t),sin(t)); surrogate interpolates
        # frame translation and rotates the local point through angle t.
        for k in range(101):
            u = k/100
            t = u/n
            actual = [(1+2*t)*math.cos(t),(1+2*t)*math.sin(t)]
            surrogate = [math.cos(t)+u*(2/n)*math.cos(1/n),
                         math.sin(t)+u*(2/n)*math.sin(1/n)]
            assert math.dist(actual,surrogate) <= coefficient/n**2+1e-12


@pytest.mark.parametrize('multiplier', [-2.0, 0.0, .5, 3.0])
def test_affine_mimic_bound_matches_compas_fk(multiplier):
    from compas_robots import RobotModel, Configuration
    from compas_robots.model import Joint, Mimic
    from compas.geometry import Frame, Point
    model = RobotModel('mimic_fixture')
    root = model.add_link('root')
    tip = model.add_link('tip')
    model.add_joint('dependent', Joint.CONTINUOUS, root, tip,
                    origin=Frame.worldXY(), axis=[0, 0, 1],
                    mimic=Mimic('driver', multiplier, .1))
    record = dict(name='tip', origin_radius=1.0, link=model.get_link_by_name('tip'))
    start = Configuration([0], [Joint.CONTINUOUS], ['driver'])
    end = Configuration([.2], [Joint.CONTINUOUS], ['driver'])
    coefficient, angular = link_chord_coefficients(model, [record], start, end)['link:tip']
    assert angular == pytest.approx(abs(multiplier*.2))
    for i in range(21):
        t = i/20
        q = _interpolate_configuration(start, end, t)
        actual = Point(1, 0, 0).transformed(model.compute_transformations(q)['dependent'])
        angle = multiplier*.2*t+.1
        assert math.dist(actual, [math.cos(angle), math.sin(angle), 0]) < 1e-12
    assert coefficient == pytest.approx(angular**2/4)


def test_mimic_resolution_matches_explicit_override_and_rejects_missing_driver():
    from types import SimpleNamespace as NS
    from compas_robots.model import Mimic
    from compas_forge.motion_bounds import _joint_position
    joint = NS(name='dependent', mimic=Mimic('driver', -2, .3))
    assert _joint_position(joint, {'driver': .2}) == pytest.approx(-.1)
    assert _joint_position(joint, {'driver': .2, 'dependent': .7}) == .7
    with pytest.raises(ValueError, match='direct mimic driver'):
        _joint_position(joint, {})
    joint.mimic.multiplier = math.inf
    with pytest.raises(ValueError, match='finite'):
        _joint_position(joint, {'driver': .2})
