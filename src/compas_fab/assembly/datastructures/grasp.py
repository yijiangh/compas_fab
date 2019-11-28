from compas.geometry import Transformation, Frame
from compas.geometry import distance_point_point


class Grasp(object):
    def __init__(self, object_index=None, grasp_id=None, approach=None, attach=None, retreat=None):
        self._object_index = object_index # brick index
        self._grasp_id = grasp_id # grasp id
        self._object_from_approach_frame = approach # compas Frame
        self._object_from_attach_frame = attach # compas Frame
        self._object_from_retreat_frame = retreat # compas Frame

        self._object_from_approach_pb_pose = None # compas Frame
        self._object_from_attach_pb_pose = None # compas Frame
        self._object_from_retreat_pb_pose = None # compas Frame

    @classmethod
    def from_frames(cls, object_index, grasp_id, world_from_obj,
        world_from_grasp, world_from_approach, world_from_retreat=None):
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
            Transformation.concatenated(world_from_obj_tf.inverse(), world_from_approach_tf))
        object_from_grasp_frame = Frame.from_transformation(\
            Transformation.concatenated(world_from_obj_tf.inverse(), world_from_grasp_tf))
        object_from_retreat_frame = Frame.from_transformation(\
            Transformation.concatenated(world_from_obj_tf.inverse(), world_from_retreat_tf))
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

    def get_object_from_approach_frame(self, get_pb_pose=False):
        if get_pb_pose:
            if not self._object_from_approach_pb_pose:
                from compas_fab.backends.pybullet import pb_pose_from_Frame
                self._object_from_approach_pb_pose = pb_pose_from_Frame(self._object_from_approach_frame)
            return self._object_from_approach_pb_pose
        else:
            return self._object_from_attach_frame

    def get_object_from_attach_frame(self, get_pb_pose=False):
        if get_pb_pose:
            if not self._object_from_attach_pb_pose:
                from compas_fab.backends.pybullet import pb_pose_from_Frame
                self._object_from_attach_pb_pose = pb_pose_from_Frame(self._object_from_attach_frame)
            return self._object_from_attach_pb_pose
        else:
            return self._object_from_attach_frame

    def get_object_from_retreat_frame(self, get_pb_pose=False):
        if get_pb_pose:
            if not self._object_from_retreat_pb_pose:
                from compas_fab.backends.pybullet import pb_pose_from_Frame
                self._object_from_retreat_pb_pose = pb_pose_from_Frame(self._object_from_retreat_frame)
            return self._object_from_retreat_pb_pose
        else:
            return self._object_from_retreat_frame

    # --------------
    # rescaling
    # --------------

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
