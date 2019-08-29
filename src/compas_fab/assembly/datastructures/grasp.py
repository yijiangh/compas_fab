from compas.geometry import Transformation, Frame

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
        pass

    def __repr__(self):
        return '{}(body index #{}, grasp id #{})'.format(self.__class__.__name__, self._object_index, self._grasp_id)
