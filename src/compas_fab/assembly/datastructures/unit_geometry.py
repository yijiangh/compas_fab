import os
import random
import warnings

from compas_fab.assembly.datastructures.grasp import Grasp
from compas_fab.assembly.datastructures.utils import obj_name
from compas.geometry import Frame
from compas.datastructures import Mesh

__all__ = ['UnitGeometry']


class UnitGeometry(object):
    """container class for a discrete object's mesh, initial/goal poses, and grasp information

    Parameters
    ----------
    object : [type]
        [description]

    Returns
    -------
    [type]
        [description]
    """
    def __init__(self, name='', meshes=None, bodies=None,
                 initial_frames=None, goal_frames=None,
                 pick_grasps=None, place_grasps=None,
                 initial_supports=None, goal_supports=None, parent_frame=None):
        # mesh is transformed to the world origin, on the user side
        self._name = name
        self._meshes = meshes or []
        self._bodies = bodies or []
        self._parent_frame =  parent_frame or Frame.worldXY()
        self._initial_frames = initial_frames or [Frame.worldXY()]
        self._initial_pb_poses = []
        self._goal_frames = goal_frames or [Frame.worldXY()]
        self._goal_pb_poses = []
        self._pick_grasps = pick_grasps or []
        self._place_grasps = place_grasps or []
        # supports are for specifying "allowed collisions" during grasping operation, not in use now
        self._initial_supports = initial_supports or []
        self._goal_supports = goal_supports or []

    @property
    def name(self):
        return self._name

    # ----------------------------------------
    # geometry frame properties
    # ----------------------------------------

    @property
    def parent_frame(self):
        """world_from_parent frame, all the other frame attributes are relative to this parent frame.

        Returns
        -------
        compas Frame
            [description]
        """
        return self._parent_frame

    @property
    def initial_frame(self):
        """Randomly choose one initial frame.

        Returns
        -------
        compas Frame
            [description]
        """
        return random.choice(self.initial_frames)

    @property
    def initial_frames(self):
        """world_from_pick_poses
        In a picknplace application, initial frame refers to the geometry's pose before its being picked up (usually pose on the material rack).

        Returns
        -------
        list of compas Frame
            [description]
        """
        return self._initial_frames

    @initial_frames.setter
    def initial_frames(self, frames_):
        assert isinstance(frames_, list)
        self._initial_frames = frames_

    def get_initial_frames(self, get_pb_pose=False):
        """[summary]

        Parameters
        ----------
        get_pb_pose : bool, optional
            if True, all frames will be converted to pybullet_planning poses, by default False

        Returns
        -------
        [type]
            [description]
        """
        if not get_pb_pose:
            return self.initial_frames
        else:
            if len(self._initial_pb_poses) == 0:
                from compas_fab.backends.pybullet import pb_pose_from_Frame
                self._initial_pb_poses = [pb_pose_from_Frame(fr) for fr in self.initial_frames]
            return self._initial_pb_poses

    @property
    def goal_frame(self):
        return random.choice(self.goal_frames)

    @property
    def goal_frames(self):
        """world_from_placed_poses
        In a picknplace context, this refers to the poses when the geometry is placed in the target design configuration.

        Returns
        -------
        [type]
            [description]
        """
        return self._goal_frames

    @goal_frames.setter
    def goal_frames(self, frames):
        assert isinstance(frames, list)
        self._goal_frames = frames

    def get_goal_frames(self, get_pb_pose=False):
        if not get_pb_pose:
            return self.goal_frames
        else:
            if len(self._goal_pb_poses) == 0:
                from compas_fab.backends.pybullet import pb_pose_from_Frame
                self._goal_pb_poses = [pb_pose_from_Frame(fr) for fr in self.goal_frames]
            return self._goal_pb_poses

    # ----------------------------------------
    # geometry mesh/body properties
    # ----------------------------------------

    @property
    def meshes(self):
        """[summary]

        Returns
        -------
        list of compas Mesh
            [description]
        """
        if not self._meshes:
            warnings.warn('no mesh is specified in the unit_geometry.')
        return self._meshes

    @meshes.setter
    def meshes(self, meshes_):
        self._meshes = meshes_
        self._bodies = [] # reset bodies

    @property
    def pybullet_bodies(self):
        """convert meshes to pybullet bodies

        Returns
        -------
        [type]
            [description]
        """
        if not self._bodies:
            # warnings.warn('no pybullet bodies is specified in the unit_geometry, generating from meshes')
            from compas_fab.backends.pybullet import convert_mesh_to_pybullet_body
            for m in self.meshes:
                self._bodies.append(convert_mesh_to_pybullet_body(m))
        return self._bodies

    def clone_pybullet_bodies(self):
        # TODO: this is temporal, workaround for `clone_body` method's bug for obj-created body
        from compas_fab.backends.pybullet import convert_mesh_to_pybullet_body
        cloned_bodies = []
        for m in self.meshes:
            cloned_bodies.append(convert_mesh_to_pybullet_body(m))
        return cloned_bodies

    # ! To make sure mesh-pb body correspondance, we only allow meshes input to specify geometries
    # @pybullet_bodies.setter
    # def pybullet_bodies(self, bodies_):
    #     assert isinstance(bodies_, list)
    #     self._bodies = bodies_

    # ----------------------------------------
    # Grasp properties
    # ----------------------------------------

    @property
    def pick_grasps(self):
        """A list of Grasps for the picking operation.

        Returns
        -------
        list of .Grasp
            [description]
        """
        return self._pick_grasps

    @pick_grasps.setter
    def pick_grasps(self, input_grasps):
        self._pick_graps = input_grasps

    @property
    def place_grasps(self):
        """[summary]

        Returns
        -------
        list of .Grasp
            [description]
        """
        return self._place_grasps

    @place_grasps.setter
    def place_grasps(self, input_grasps):
        self._place_graps = input_grasps

    # --------------------------------------------------------------------------
    # utils
    # --------------------------------------------------------------------------

    def rescale(self, scale):
        """rescale all the frames and meshes according to a scale conversion rate.

        Parameters
        ----------
        scale : float
            unit conversion rate, e.g. 1.0, 1e-3.

        Returns
        -------
        No return
            [description]
        """
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
        for gf in self._initial_frames:
            scale_frame(gf, scale)
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
        if not mesh_path:
            return file_names
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
       return 'UG {}: #mesh: {}, #body: {}, #init_frame: {}, #goal_frame: {}, #pick_grasp: {}, #place_grasp: {}'.format( \
           self.name, len(self.meshes), len(self.pybullet_bodies),
           len(self.initial_frames), len(self.goal_frames), len(self.pick_grasps), len(self.place_grasps))
