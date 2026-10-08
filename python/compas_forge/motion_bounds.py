"""Exact-arithmetic derivative bounds for COMPAS Robots' fixed-axis products.

Restricted to fixed/revolute/continuous/prismatic joints, linear joint paths,
and the unmodified COMPAS Robots transformation contract. See ROBUSTNESS.md.
"""
import math
from compas_robots.model import Joint


def _joint_position(joint, values):
    """Match RobotModel.compute_transformations, including explicit overrides.

    COMPAS resolves only a directly supplied mimic driver, not recursive mimic
    chains. Reject missing drivers rather than silently bounding a different FK.
    """
    if joint.name in values:
        return values[joint.name]
    if joint.mimic is not None and joint.mimic.joint in values:
        if not all(math.isfinite(v) for v in (joint.mimic.multiplier, joint.mimic.offset)):
            raise ValueError('mimic coefficients must be finite')
        return joint.mimic.calculate_position(values[joint.mimic.joint])
    raise ValueError('articulated bounds require a complete robot configuration or a direct mimic driver')


def _chain_envelope(model, link, start, end, envelope):
    """Apply a joint chain to an existing (B0,B1,B2,angular) envelope."""
    qa = dict(zip(start.joint_names,start.joint_values))
    qb = dict(zip(end.joint_names,end.joint_values))
    b0,b1,b2,angular = envelope
    joint = link.parent_joint if link is not None else None
    visited = set()
    while joint is not None:
        if joint.name in visited:
            raise ValueError('cyclic joint graph')
        visited.add(joint.name)
        if joint.type not in (Joint.FIXED,Joint.REVOLUTE,Joint.CONTINUOUS,Joint.PRISMATIC):
            raise NotImplementedError('unsupported joint type for articulated bounds')
        if joint.type != Joint.FIXED:
            a,b = _joint_position(joint,qa),_joint_position(joint,qb)
            if not math.isfinite(a) or not math.isfinite(b):
                raise ValueError('nonfinite joint configuration')
            if joint.type == Joint.REVOLUTE and joint.limit is not None:
                if min(a,b) < joint.limit.lower or max(a,b) > joint.limit.upper:
                    raise ValueError('bounds require unclamped in-limit revolute motion')
            delta = abs(b-a)
            axis_length = math.hypot(*joint.current_axis.vector)
            if not math.isfinite(axis_length) or axis_length <= 0:
                raise ValueError('joint axis must be finite and nonzero')
            if joint.type == Joint.PRISMATIC:
                if joint.limit is None or min(a,b) < joint.limit.lower or max(a,b) > joint.limit.upper:
                    raise ValueError('bounds require unclamped in-limit prismatic motion')
                b0 += max(abs(a),abs(b))*axis_length
                b1 += delta*axis_length
            else:
                origin = math.hypot(*joint.current_origin.point)
                b2 += 2*delta*b1+delta*delta*(b0+origin)
                b1 += delta*(b0+origin)
                b0 += 2*origin
                angular += delta
        joint = model.get_link_by_name(joint.parent.link).parent_joint
    return b0,b1,b2,angular


def tool_link_chord_coefficient(tool, record, start, end, robot, parent_link,
                                robot_start, robot_end, fixed_offset=0.0, root_angle=0.0):
    """Bound C*T_robot(q)*F*T_tool(u)*p, or a freely moving rigid tool base.

    A fixed F adds only ||translation(F)|| to B0. Applying the robot chain after
    the tool chain retains mixed derivative terms; summing two separate error
    bounds would miss those terms. Tool TCP remains the fixed COMPAS ToolModel.frame.
    """
    radius = record['origin_radius']
    b0,b1,b2,angular = _chain_envelope(tool,record['link'],start,end,(radius,0.0,0.0,0.0))
    if parent_link is not None:
        b0 += fixed_offset
        b0,b1,b2,angular = _chain_envelope(robot,parent_link,robot_start,robot_end,(b0,b1,b2,angular))
    elif root_angle:
        b2 += 2*root_angle*b1+root_angle*root_angle*b0
        angular += root_angle
    coefficient = (b2+radius*angular*angular)/8.0
    if not math.isfinite(coefficient):
        raise ValueError('nonfinite coupled-tool motion bound')
    return coefficient,angular


def link_chord_coefficients(model, records, start, end):
    """Return E and angular variation per link; n subdivisions give error E/n².

    This bounds the deviation of an actual articulated material point from the
    endpoint rigid surrogate, not physical calibration or controller error.
    """
    result = {}
    for record in records:
        radius = record['origin_radius']
        _,_,b2,angular = _chain_envelope(model,record['link'],start,end,(radius,0.0,0.0,0.0))
        surrogate_radius = record.get('surrogate_radius', radius)
        coefficient = (b2+surrogate_radius*angular*angular)/8.0
        if not math.isfinite(coefficient):
            raise ValueError('nonfinite articulated motion bound; rescale inputs')
        result[record.get('key', 'link:'+record['name'])] = (coefficient,angular)
    return result
