"""An abstract geometry object that can be made as parametric geometry class
or a determinate mesh object

"""
# TODO: this dependency is only required when getting pybullet_bodies

class UnitGeometry(object):
    def __init__(self, name, mesh=None, body=None,
                 initial_frame=None, goal_frame=None,
                 grasps=[], initial_supports=[], goal_supports=[]):
        self._name = name
        self._mesh = mesh
        self._body = body
        self._initial_frame = initial_frame
        self._goal_frame = goal_frame
        self._grasps = grasps
        self._initial_supports = initial_supports
        self._goal_supports = goal_supports

    @property
    def name(self):
        return self._name

    @property
    def pybullet_bodies(self):
        # TODO: if no body is stored, create from compas mesh
        return self._body

    @pybullet_bodies.setter
    def pybullet_bodies(self, body):
        self._body = body

    @property
    def mesh(self):
        # TODO: if no mesh is stored, but pybullet body is assigned
        # create from pybullet body
        return self._mesh

    @mesh.setter
    def mesh(self, input_mesh):
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
