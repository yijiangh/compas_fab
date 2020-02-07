from compas.geometry import Frame, Transformation

__all__ = ['pb_pose_from_Frame',
           'pb_pose_from_Transformation',
           'Frame_from_pb_pose',
           'Frame_from_pos_rot']

def pb_pose_from_Frame(frame):
    """Convert compas.Frame to (point, quat).

    Parameters
    ----------
    frame : compas Frame

    Returns
    -------
    tuple
        (point, euler),
        where point: [x, y, z]
              quat:  [roll, pitch, yaw]
    """
    from pybullet_planning import quat_from_euler
    point = [frame.point.x, frame.point.y, frame.point.z]
    euler = frame.euler_angles()
    return (point, quat_from_euler(euler))


def pb_pose_from_Transformation(tf):
    """Convert compas.Transformation to (point, quat).

    Parameters
    ----------
    tf : compas Transformation

    Returns
    -------
    tuple
        (point, euler),
        where point: [x, y, z]
              quat:  [roll, pitch, yaw]
    """
    from pybullet_planning import quat_from_matrix
    point = [tf[i, 3] for i in range(3)]
    quat = quat_from_matrix([tf[i, :3] for i in range(3)])
    return (point, quat)


def Frame_from_pb_pose(pose):
    """Convert (point, quat) to compas.Frame

    Parameters
    ----------
    pose : ([x,y,z], [roll, pitch, yaw])

    Returns
    -------
    frame : compas Frame
    """
    from pybullet_planning import euler_from_quat
    frame = Frame.from_euler_angles(euler_from_quat(pose[1]), point=pose[0])
    return frame

def Frame_from_pos_rot(pos, rot):
    """Convert (point, rotation matrix) to compas.Frame

    Parameters
    ----------
    pos : list of float
        [x, y, z]
    rot : list of lists
        row-major 3x3 rotation matrix (same format as the outcome of np.array().tolist())

    Returns
    -------
    frame : compas Frame
    """
    from pybullet_planning import quat_from_matrix
    return Frame.from_euler_angles(quat_from_matrix(rot), point=pos)
