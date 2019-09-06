from compas.geometry import Transformation, Frame
from compas.geometry import distance_point_point

AVAILABLE_INTERPOLATION_FNS = {
    'straight_line_keep_orientation' : get_straight_line_interpolate_fn,
    'slerp' : None,
}

# TODO: slerp interpolation

def get_straight_line_interpolate_fn(disc_len, mount_link_from_tcp=None):
    """built-in pose intepolation function, generate a list of frames connecting from frame_0
    to frame_1, use disc_len as division length. If mount_link_from_tcp is given, additional transformation
    will be performed.

    Note: this function assume that frame_0 and frame_1 have the same orientation, only the positions are
    different.

    Parameters
    ----------
    disc_len : float
        length for interpolation division, unit in meter
    mount_link_from_tcp : compas transformation, optional
        mount link from tcp transformation, by default None

    Returns
    -------
    [type]
        [description]
    """

    def interp_fn(frame_0, frame_1):
        """intepolation function

        Parameters
        ----------
        frame_0 : compas Frame
            starting pose
        frame_1 : compas Frame
            end pose
        """
        # TODO: check p1 should have the same orientation of p2
        assert frame_0.euler_angles() == frame_1.euler_angles()
        p0 = frame_0.point
        p1 = frame_1.point
        e_len = distance_point_point(p0, p1)

        # advance = np.append(np.arange(0, e_len, disc_len), e_len)
        advance = [0]
        while max(advance)+disc_len < e_len:
            advance.append(advance[-1] + disc_len)
        if abs(advance[-1] - advance[-2]) < 1e-5:
            advance = advance.pop(-2)

        world_from_tcps = [Frame(point=(p0 + (t/e_len)**(p1-p0)), xaxis=frame_0.xaxis, yaxis=frame_0.yaxis) for t in advance]
        if mount_link_from_tcp:
            return [Frame.from_transformation(Transformation.concatenate(
                Transformation.from_frame(world_from_tcp), mount_link_from_tcp.inverse()))
                for world_from_tcp in world_from_tcps]
        else:
            return world_from_tcps

    return interp_fn

class UnitCartesianProcess(object):
    def __init__(self, sub_process_id, unit_geometry=None,
        parent_frame=Frame.worldXY(), key_frames=[], path_interpolation_fn=None, disc_len=0.005, mount_link_from_TCP_tf=None):
        self._manipulated_unit_geometry = unit_geometry
        self._unit_assembly_process_id = sub_process_id
        self._attached_unit_geometries = []

        self._parent_frame = parent_frame
        self._key_path_frames = key_frames
        self._path_interpolation_fn = path_interpolation_fn or get_straight_line_interpolate_fn(
            disc_len, mount_link_from_tcp=mount_link_from_TCP_tf)
        self._ignored_body_pairs = [] # this is pybullet bodies for now

    @property
    def parent_frame(self):
        return self._parent_frame

    def get_to_world_frame_transform_fn(self):
        world_from_parent_tf = Transformation.from_frame(self.parent_frame)
        def transf_to_world_fn(parent_from_target):
            return Frame.from_transformation(\
                Transformation.concatenate(world_from_parent_tf, Transformation.from_frame(parent_from_target)))
        return transf_to_world_fn

    @property
    def key_path_frames(self):
        return self._key_path_frames

    @property
    def key_path_pb_poses(self):
        from compas_fab.backends.pybullet import pb_pose_from_Frame
        return [pb_pose_from_Frame(frame) for frame in self.key_path_frames]

    @property
    def path_frames(self):
        key_frames = self.key_path_frames
        full_frames = []
        for i in range(len(key_frames)-1):
            full_frames.append(self.path_pose_interpolation_fn(key_frames[i], key_frames[i+1]))
        return full_frames

    @property
    def path_pb_poses(self):
        full_frames = self.path_frames
        from compas_fab.backends.pybullet import pb_pose_from_Frame
        return [pb_pose_from_Frame(frame) for frame in full_frames]

    @property
    def path_pose_interpolation_fn(self):
        return self._path_interpolation_fn

    @path_pose_interpolation_fn.setter
    def path_pose_interpolation_fn(self, interp_fn):
        self._path_interpolation_fn = interp_fn

    @property
    def ignored_body_pairs(self):
        return self._ignored_body_pairs

    @ignored_body_pairs.setter
    def ignored_body_pairs(self, body_pairs):
        self._ignored_body_pairs = body_pairs

    def get_collision_fn(self, robot, ik_joints, obstacles, ee_attachs=[]):
        from choreo.choreo_utils import get_collision_fn
        return get_collision_fn(robot, ik_joints, obstacles,
                                attachments=ee_attachs + self._attached_unit_geometries,
                                self_collisions=True, ignored_pairs=self.ignored_body_pairs)


class CartesianProcess(object):
    def __init__(self, object_index, process_id):
        self._assembly_process_id = process_id # brick index
        self._sub_process_dict = {}

    def get_collision_fns(self):
        return

    def get_all_ee_poses(self):
        # pb poses only for now
        pass


class SemiConstrainedCartesianProcess(CartesianProcess):
    def __init__(self):
        self._path_poses_gen_fn = None

    def path_frame_gen_fn(self):
        return self._path_poses_gen_fn


# TODO: now variation in orientation only, variation in TCP point as well?
class DiscreteSemiConstrainedCartesianProcess(SemiConstrainedCartesianProcess):
    def __init__(self):
        pass

    def set_path_frame_gen_fn_from_orientation_list(self, rotation_list):
        def enumerate_gen_fn():
            pass
        self._path_poses_gen_fn = enumerate_gen_fn

# TODO: abstract class for representing robotic primitives
class Grasp(object):
    def __init__(self, object_index=None, grasp_id=None, approach=None, attach=None, retreat=None):
        self._object_index = object_index # brick index
        self._grasp_id = grasp_id # grasp id
        self._object_from_approach_frame = approach # compas Frame
        self._object_from_attach_frame = attach # compas Frame
        self._object_from_retreat_frame = retreat # compas Frame

    @classmethod
    def from_frames(cls, object_index, grasp_id, world_from_obj, world_from_grasp, world_from_approach, world_from_retreat=None):
        """construct a grasp instance from grasp, approach and retreat in the same coordinate system
        (does not need to be world)

        Parameters
        ----------
        world_from_obj : compas.geometry.Frame
            [description]
        world_from_grasp : compas.geometry.Frame
            [description]
        world_from_approach : compas.geometry.Frame
            [description]
        world_from_retreat : compas.geometry.Frame, optional
            [description]
        """
        if not world_from_retreat:
            world_from_retreat = world_from_approach

        world_from_obj_tf = Transformation.from_frame(world_from_obj)
        world_from_approach_tf = Transformation.from_frame(world_from_approach)
        world_from_grasp_tf = Transformation.from_frame(world_from_grasp)
        world_from_retreat_tf = Transformation.from_frame(world_from_retreat)

        object_from_approach_frame = Frame.from_transformation(\
            Transformation.concatenate(world_from_obj_tf.inverse(), world_from_approach_tf))
        object_from_grasp_frame = Frame.from_transformation(\
            Transformation.concatenate(world_from_obj_tf.inverse(), world_from_grasp_tf))
        object_from_retreat_frame = Frame.from_transformation(\
            Transformation.concatenate(world_from_obj_tf.inverse(), world_from_retreat_tf))
        return cls(object_index, grasp_id, object_from_approach_frame, object_from_grasp_frame, object_from_retreat_frame)

    # --------------
    # json serialization
    # --------------

    @classmethod
    def from_data(cls, data):
        return cls(data['object_index'], data['grasp_id'],
            Frame.from_data(data['object_from_approach_frame']),
            Frame.from_data(data['object_from_attach_frame']),
            Frame.from_data(data['object_from_retreat_frame']))

    def to_data(self):
        data = {}
        data['object_index'] = self._object_index
        data['grasp_id'] = self._grasp_id
        data['object_from_approach_frame'] = self._object_from_approach_frame.to_data()
        data['object_from_attach_frame'] = self._object_from_attach_frame.to_data()
        data['object_from_retreat_frame'] = self._object_from_retreat_frame.to_data()
        return data

    # --------------
    # pybullet pose conversion
    # --------------

    @property
    def object_from_approach_pb_pose(self):
        try:
            from compas_fab.backends.pybullet import pb_pose_from_Frame
            return pb_pose_from_Frame(self._object_from_approach_frame)
        except ImportError:
            return None

    @property
    def object_from_attach_pb_pose(self):
        try:
            from compas_fab.backends.pybullet import pb_pose_from_Frame
            return pb_pose_from_Frame(self._object_from_attach_frame)
        except ImportError:
            return None

    @property
    def object_from_retreat_pb_pose(self):
        try:
            from compas_fab.backends.pybullet import pb_pose_from_Frame
            return pb_pose_from_Frame(self._object_from_retreat_frame)
        except ImportError:
            return None

    # --------------
    # Cartesian path generation
    # --------------

    def get_object_from_approach_to_attach_path(self, disc=10, as_pb_pose=False):
        """Get linear interpolation between the approach and attach poses,
        in object_from_x coordinate.

        Parameters
        ----------
        disc : int, optional
            number of interpolated path points, by default 10
        as_pb_pose : bool, optional
            output pybullet Pose or not, by default False
        """
        pass

    def get_object_from_attach_to_retreat_path(self, disc=10, as_pb_pose=False):
        """Get linear interpolation between the attach and retreat poses,
        in object_from_x coordinate.

        Parameters
        ----------
        disc : int, optional
            number of interpolated path points, by default 10
        as_pb_pose : bool, optional
            output pybullet Pose or not, by default False
        """
        pass

    def rescale(self, scale):
        def scale_frame(fr, scale):
            fr.point *= scale
        scale_frame(self._object_from_approach_frame, scale)
        scale_frame(self._object_from_attach_frame, scale)
        scale_frame(self._object_from_retreat_frame, scale)

    # def __eq__(self, other):
    #     if self._object_index == other._object_index and \
    #         self._grasp_id == other._grasp_id and \
    #         self._object_from_approach_frame == other._object_from_approach_frame and \
    #         self._object_from_attach_frame == other._object_from_approach_frame and \
    #         self._object_from_retreat_frame == other._object_from_approach_frame:
    #         return True
    #     else:
    #         return False

    def __repr__(self):
        return '{}(body index #{}, grasp id #{})'.format(self.__class__.__name__, self._object_index, self._grasp_id)
