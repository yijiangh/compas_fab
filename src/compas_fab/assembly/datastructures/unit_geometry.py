"""An abstract geometry object that can be made as parametric geometry class
or a determinate mesh object

"""
import os
from .grasp import Grasp
from .utils import obj_name
from compas.geometry import Frame
from compas.datastructures import Mesh

__all__ = ['UnitGeometry']


class UnitGeometry(object):
    """Unit geometry ...

    """
    def __init__(self, name='', meshes=[], bodies=[],
                 initial_frames=[], goal_frames=[],
                 pick_grasps=[], place_grasps=[],
                 initial_supports=[], goal_supports=[], parent_frame=None):
        # mesh is transformed to the world origin, on the user side
        self._name = name
        self._meshes = meshes
        self._bodies = bodies
        self._parent_frame =  parent_frame or Frame.worldXY()
        self._initial_frames = initial_frames or [Frame.worldXY()]
        self._goal_frames = goal_frames or [Frame.worldXY()]
        self._pick_grasps = pick_grasps
        self._place_grasps = place_grasps
        self._initial_supports = initial_supports
        self._goal_supports = goal_supports

    @property
    def name(self):
        return self._name

    # --------------------------------------------------------------------------
    # geometry frame properties
    # --------------------------------------------------------------------------

    @property
    def parent_frame(self):
        return self._parent_frame

    @property
    def initial_frames(self):
        return self._initial_frames

    @initial_frames.setter
    def initial_frames(self, frames):
        assert isinstance(frames, list)
        self._initial_frames = frames

    @property
    def initial_pb_poses(self):
        try:
            from compas_fab.backends.pybullet import pb_pose_from_Frame
            return pb_pose_from_Frame(self._initial_frames)
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
    # geometry mesh/body properties
    # --------------------------------------------------------------------------

    @property
    def meshes(self):
        # TODO: if no mesh is stored, but pybullet body is assigned
        # create from pybullet body
        return self._meshes

    @meshes.setter
    def meshes(self, input_meshes):
        self._meshes = input_meshes

    @property
    def pybullet_bodies(self):
        # TODO: if no body is stored, create from compas mesh
        return self._bodies

    @pybullet_bodies.setter
    def pybullet_bodies(self, body):
        self._bodies = body

    # --------------------------------------------------------------------------
    # Grasp properties
    # --------------------------------------------------------------------------

    # For now, this unit geometry class is binded to grasp operation
    # these attributes are specific to picknplace applications
    # we should plan to replace it with a more abstract class that
    # represents geometry's relationship with end effector
    # can be a class called `CartesianOperation`, and Grasp, Extrusion can inherit from it

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

        for i in range(len(self._meshes)):
            scale_mesh(self.meshes[i], scale)
        scale_frame(self._parent_frame, scale)
        scale_frame(self._initial_frames, scale)
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
        for sub_mesh_id, cm in enumerate(self.meshes):
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
        data['initial_frames'] = [in_f.to_data() for in_f in self._initial_frames]
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
        return cls(data['name'], meshes=meshes,
                 initial_frames=[Frame.from_data(in_data) for in_data in data['initial_frames']],
                 goal_frames=[Frame.from_data(gf_data) for gf_data in data['goal_frames']],
                 pick_grasps=pick_grasps, place_grasps=place_grasps, parent_frame=Frame.from_data(data['parent_frame']))

    # --------------
    # repr
    # --------------
    def __repr__(self):
       return 'UG: #mesh: {}, #body: {}, #init_frame: {}, #goal_frame: {}, #pick_grasp: {}, #place_grasp: {}'.format( \
           len(self.meshes), len(self.pybullet_bodies),
           len(self.initial_frames), len(self.goal_frames), len(self.pick_grasps), len(self.place_grasps))
