"""An abstract geometry object that can be made as parametric geometry class
or a determinate mesh object

"""
import os
from .grasp import Grasp
from .utils import obj_name
from compas.geometry import Frame
from compas.datastructures import Mesh

class UnitGeometry(object):
    def __init__(self, name='', mesh=[], body=[],
                 initial_frame=None, goal_frame=None,
                 grasps=[], initial_supports=[], goal_supports=[], parent_frame=None):
        self._name = name
        self._mesh = mesh
        self._body = body
        self._parent_frame =  parent_frame or Frame.worldXY()
        self._initial_frame =  initial_frame or Frame.worldXY()
        self._goal_frame = goal_frame or Frame.worldXY()
        self._grasps = grasps
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
    def grasps(self):
        """obj_from_grasp_poses
        """
        return self._grasps

    @grasps.setter
    def grasps(self, input_grasps):
        self._graps = input_grasps

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
        # TODO: if no body is stored, create from compas mesh
        return self._goal_frame

    @goal_frame.setter
    def goal_frame(self, frame):
        self._goal_frame = frame

    @property
    def goal_pb_pose(self):
        try:
            from compas_fab.backends.pybullet import pb_pose_from_Frame
            return pb_pose_from_Frame(self._goal_frame)
        except ImportError:
            return None

    # @property
    # def world_from_element_place_pose(self):
    #     return self._world_from_element_place_pose

    # @world_from_element_place_pose.setter
    # def world_from_element_place_pose(self, pose):
    #     self._world_from_element_place_pose = pose

    # @property
    # def world_from_element_pick_pose(self):
    #     return self._world_from_element_pick_pose

    # @world_from_element_pick_pose.setter
    # def world_from_element_pick_pose(self, pose):
    #     self._world_from_element_pick_pose = pose

    # @property
    # def obj_from_grasp_poses(self):
    #     return self._obj_from_grasp_poses

    # @obj_from_grasp_poses.setter
    # def obj_from_grasp_poses(self, grasp_poses):
    #     self._obj_from_grasp_poses = grasp_poses

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
        scale_frame(self._goal_frame, scale)
        for i in range(len(self._grasps)):
            self._grasps[i].rescale(scale)

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
        data['goal_frame'] = self._goal_frame.to_data()
        data['grasps'] = [g.to_data() for g in self._grasps]

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
        grasps = [Grasp.from_data(g_data) for g_data in data['grasps']]
        return cls(data['name'], mesh=meshes,
                 initial_frame=Frame.from_data(data['initial_frame']), goal_frame=Frame.from_data(data['goal_frame']),
                 grasps=grasps, parent_frame=Frame.from_data(data['parent_frame']))
