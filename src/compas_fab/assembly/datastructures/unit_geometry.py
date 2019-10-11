"""An abstract geometry object that can be made as parametric geometry class
or a determinate mesh object

"""
import os
from .grasp import Grasp
from .utils import obj_name
from compas.geometry import Frame
from compas.datastructures import Mesh

class UnitGeometry(object):
    """Unit geometry ...

    Note:
    We assume in most use cases, there is only one designated goal pose of the geometry from the users.
    The initial poses, however, might vary depends on the grasp poses, espcially in the case when the
    unit geometry has symmetric properties.

    For example, in the case of a beam with 4 identical side faces (or a rod), a user might designate
    multiple grasp poses in **goal** pose, but only one grasp pose for **initial** pose, which is limited
    because of the material rack's design.

    """
    def __init__(self, name='', mesh=[], body=[],
                 initial_frame=None, goal_frames=[],
                 pick_grasps=[], place_grasps=[],
                 initial_supports=[], goal_supports=[], parent_frame=None):
        # mesh is transformed to the world origin, on the user side
        self._name = name
        self._mesh = mesh
        self._body = body
        self._parent_frame =  parent_frame or Frame.worldXY()
        self._initial_frame = initial_frame or Frame.worldXY()
        # goal frames is a list to indicate symmetry in the unit geometry
        self._goal_frames = goal_frames or [Frame.worldXY()]
        self._pick_grasps = pick_grasps
        self._place_grasps = place_grasps
        self._initial_supports = initial_supports
        self._goal_supports = goal_supports

    @property
    def name(self):
        return self._name

    @property
    def parent_frame(self):
        return self._parent_frame

    @property
    def pybullet_bodies(self):
        # TODO: if no body is stored, create from compas mesh
        return self._body

    @pybullet_bodies.setter
    def pybullet_bodies(self, body):
        self._body = body

    @property
    def mesh(self):
        """ this is a list!!! """
        # TODO: if no mesh is stored, but pybullet body is assigned
        # create from pybullet body
        return self._mesh

    @mesh.setter
    def mesh(self, input_mesh):
        """ this is a list!!! """
        self._mesh = input_mesh

    @property
    def pick_grasps(self):
        """obj_from_grasp_poses
        """
        return self._pick_grasps

    @pick_grasps.setter
    def pick_grasps(self, input_grasps):
        self._pick_graps = input_grasps

    @property
    def place_grasps(self):
        """obj_from_grasp_poses
        """
        return self._place_grasps

    @place_grasps.setter
    def place_grasps(self, input_grasps):
        self._place_graps = input_grasps

    @property
    def initial_frame(self):
        """world_from_element_pick_pose
        """
        return self._initial_frame

    @initial_frame.setter
    def initial_frame(self, frame):
        self._initial_frame = frame

    @property
    def initial_pb_pose(self):
        try:
            from compas_fab.backends.pybullet import pb_pose_from_Frame
            return pb_pose_from_Frame(self._initial_frame)
        except ImportError:
            return None

    @property
    def goal_frame(self):
        """world_from_element_place pose
        """
        return self._goal_frames[0]

    @property
    def goal_frames(self):
        """world_from_element_place pose
        """
        # TODO: if no body is stored, create from compas mesh
        return self._goal_frames

    @goal_frames.setter
    def goal_frames(self, frames):
        assert isinstance(frames, list)
        self._goal_frames = frames

    @property
    def goal_pb_pose(self):
        try:
            from compas_fab.backends.pybullet import pb_pose_from_Frame
            return pb_pose_from_Frame(self.goal_frame)
        except ImportError:
            return None

    @property
    def goal_pb_poses(self):
        try:
            from compas_fab.backends.pybullet import pb_pose_from_Frame
            return [pb_pose_from_Frame(gf) for gf in self.goal_frames]
        except ImportError:
            return None

    # --------------------------------------------------------------------------
    # attributes
    # --------------------------------------------------------------------------

    def centroid(self):
        raise NotImplementedError

    # --------------------------------------------------------------------------
    # utils
    # --------------------------------------------------------------------------
    def rescale(self, scale):
        def scale_mesh(m, scale):
            for key, v in m.vertex.items():
                for k in v.keys():
                    m.vertex[key][k] *= scale
            return m
        def scale_frame(fr, scale):
            fr.point *= scale

        for i in range(len(self._mesh)):
            scale_mesh(self.mesh[i], scale)
        scale_frame(self._parent_frame, scale)
        scale_frame(self._initial_frame, scale)
        for gf in self._goal_frames:
            scale_frame(gf, scale)
        for i in range(len(self.pick_grasps)):
            self.pick_grasps[i].rescale(scale)
        for i in range(len(self.place_grasps)):
            self.place_grasps[i].rescale(scale)

    # --------------------------------------------------------------------------
    # exporters
    # --------------------------------------------------------------------------

    def to_objs(self, mesh_path):
        file_names = []
        for sub_mesh_id, cm in enumerate(self.mesh):
            if os.path.exists(mesh_path):
                file_name = os.path.join(mesh_path, obj_name(self.name, sub_mesh_id))
                cm.to_obj(file_name)
            else:
                file_name = obj_name(self.name, sub_mesh_id)
            file_names.append(file_name)
        return file_names

    def to_data(self, mesh_path=None):
        data = {}
        data['name'] = self._name

        file_names = self.to_objs(mesh_path)
        data['mesh_file_names'] = file_names

        data['parent_frame'] = self._parent_frame.to_data()
        data['initial_frame'] = self._initial_frame.to_data()
        data['goal_frames'] = [gf.to_data() for gf in self._goal_frames]
        data['pick_grasps'] = [g.to_data() for g in self.pick_grasps]
        data['place_grasps'] = [g.to_data() for g in self.place_grasps]

        # TODO:
        # self._initial_supports = initial_supports
        # self._goal_supports = goal_supports
        return data

    @classmethod
    def from_data(cls, data):
        meshes = []
        for file_name in data['mesh_file_names']:
            if os.path.exists(file_name):
                meshes.append(Mesh.from_obj(file_name))
        pick_grasps = [Grasp.from_data(g_data) for g_data in data['pick_grasps']]
        place_grasps = [Grasp.from_data(g_data) for g_data in data['place_grasps']]
        return cls(data['name'], mesh=meshes,
                 initial_frame=Frame.from_data(data['initial_frame']), goal_frames=[Frame.from_data(gf_data) for gf_data in data['goal_frames']],
                 pick_grasps=pick_grasps, place_grasps=place_grasps, parent_frame=Frame.from_data(data['parent_frame']))
