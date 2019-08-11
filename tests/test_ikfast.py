from __future__ import print_function
import os
import time
import random
import pytest
import numpy as np
from numpy.testing import assert_array_almost_equal

from compas.geometry import Frame, Translation
from compas.datastructures import Mesh, mesh_transform
from compas.robots import RobotModel

import compas_fab
from compas_fab.backends import RosClient
from compas_fab.robots import Robot as RobotClass
from compas_fab.robots import RobotSemantics
from compas_fab.robots import PlanningScene
from compas_fab.robots import CollisionMesh
from compas_fab.robots.ur5 import Robot

# pybullet interface utils
from compas_fab.backends.pybullet import attach_end_effector_geometry, \
convert_mesh_to_pybullet_body, get_TCP_pose, create_pb_robot_from_ros_urdf, \
convert_meshes_and_poses_to_pybullet_bodies, pb_pose_from_Transformation

# pybullet grasp utils
from compas_fab.backends.pybullet import plan_pickup_object, get_grasp_gen

from compas_fab.backends.ros.plugins_choreo import get_ik_tool_link_pose, \
 sample_tool_ik, best_sol

from conrob_pybullet import load_pybullet, connect, disconnect, wait_for_user, \
    LockRenderer, has_gui, get_model_info, get_pose, euler_from_quat, draw_pose, \
    get_link_pose, link_from_name, create_attachment, add_fixed_constraint, \
    create_obj, set_pose, get_sample_fn, violates_limits, joints_from_names, \
    set_joint_positions, remove_debug, get_joint_limits, WorldSaver, \
    LockRenderer, update_state, end_effector_from_body, approach_from_grasp, \
    unit_pose, approximate_as_prism, point_from_pose, multiply, quat_from_euler, \
    approximate_as_cylinder, quat_from_matrix, matrix_from_quat

from conrob_pybullet import Pose, Point, BodyPose, GraspInfo

try:
    import ikfast_ur5
except ImportError as e:
    assert False, '\x1b[6;30;43m' + '{}, please install ikfast_pybind'.format(e) + '\x1b[0m'


def test_ikfast_forward_kinematics():
    """TODO: this test_function can by pybullet-free"""
    urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
    srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')
    urdf_pkg_name = 'ur_description'

    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
    robot = RobotClass(model, semantics=semantics)

    base_link_name = robot.get_base_link_name()
    ik_joint_names = robot.get_configurable_joint_names()
    ik_tool_link_name = robot.get_end_effector_link_name()

    connect(use_gui=False)
    pb_robot = create_pb_robot_from_ros_urdf(urdf_filename, urdf_pkg_name)
    pb_ik_joints = joints_from_names(pb_robot, ik_joint_names)

    max_attempts = 20
    sample_fn = get_sample_fn(pb_robot, pb_ik_joints)

    for _ in range(max_attempts):
        # randomly sample within joint limits
        conf = sample_fn()
        # sanity joint limit violation check
        assert not violates_limits(pb_robot, pb_ik_joints, conf)

        # ikfast's FK
        fk_fn = ikfast_ur5.get_fk
        ikfast_FK_pb_pose = get_ik_tool_link_pose(fk_fn, pb_robot, ik_joint_names, base_link_name, conf)
        # print('ikfast FK sol: {}'.format(ikfast_FK_pb_pose))

        # pybullet's FK
        set_joint_positions(pb_robot, pb_ik_joints, conf)
        TCP_pb_pose = get_TCP_pose(pb_robot, ik_tool_link_name, return_pb_pose=True)
        # print('pybullet FK sol: {}'.format(TCP_pb_pose))

        assert_array_almost_equal(TCP_pb_pose[0], ikfast_FK_pb_pose[0])
        assert_array_almost_equal(TCP_pb_pose[1], ikfast_FK_pb_pose[1])


def test_ikfast_inverse_kinematics():
    """TODO: this test_function can by pybullet-free"""
    urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
    srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')
    urdf_pkg_name = 'ur_description'

    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
    robot = RobotClass(model, semantics=semantics)

    base_link_name = robot.get_base_link_name()
    ik_joint_names = robot.get_configurable_joint_names()
    ik_tool_link_name = robot.get_end_effector_link_name()

    connect(use_gui=False)
    pb_robot = create_pb_robot_from_ros_urdf(urdf_filename, urdf_pkg_name)
    pb_ik_joints = joints_from_names(pb_robot, ik_joint_names)

    max_attempts = 20
    EPS = 1e-3
    sample_fn = get_sample_fn(pb_robot, pb_ik_joints)

    for i in range(max_attempts):
        # randomly sample within joint limits
        conf = sample_fn()
        # sanity joint limit violation check
        assert not violates_limits(pb_robot, pb_ik_joints, conf)

        # ikfast's FK
        fk_fn = ikfast_ur5.get_fk
        ikfast_FK_pb_pose = get_ik_tool_link_pose(fk_fn, pb_robot, ik_joint_names, base_link_name, conf)

        if has_gui():
            print('test round #{}: ground truth conf: {}'.format(i, conf))
            handles = draw_pose(ikfast_FK_pb_pose, length=0.04)
            set_joint_positions(pb_robot, pb_ik_joints, conf)
            wait_for_user()

        # ikfast's IK
        ik_fn = ikfast_ur5.get_ik
        ik_sols = sample_tool_ik(ik_fn, pb_robot, ik_joint_names, base_link_name,
                        ikfast_FK_pb_pose, get_all=True)

        # TODO: UR robot or in general joint w/ domain over 4 pi
        # needs specialized distance function
        q_selected = sample_tool_ik(ik_fn, pb_robot, ik_joint_names, base_link_name,
                        ikfast_FK_pb_pose, nearby_conf=True)
        qsol = best_sol(ik_sols, conf, [1.]*6)
        print('q selected: {}'.format(q_selected))
        print('q best: {}'.format(qsol))

        if has_gui():
            set_joint_positions(pb_robot, pb_ik_joints, qsol)
            wait_for_user()
            for h in handles : remove_debug(h)

        if qsol is None:
            qsol = [999.]*6
        diff = np.sum(np.abs(np.array(qsol) - np.array(conf)))
        if diff > EPS:
            print(np.array(ik_sols))
            print('Best q:{}'.format(qsol))
            print('Actual:{}'.format(np.array(conf)))
            print('Diff:  {}'.format(conf - qsol))
            print('Difdiv:{}'.format((conf - qsol)/np.pi))
            assert False



def get_side_cylinder_grasps(body, under=False, tool_pose=Pose(), body_pose=unit_pose(),
                             max_width=np.inf, grasp_length=0, top_offset=0.03):
    center, (diameter, height) = approximate_as_cylinder(body, body_pose=body_pose)
    translate_center = Pose(point_from_pose(body_pose)-center)
    #x_offset = 0
    x_offset = height/2 - top_offset
    if max_width < diameter:
        return
    while True:
        theta = random.uniform(0, 2*np.pi)
        translate_rotate = ([x_offset, 0, diameter / 2 - grasp_length], quat_from_euler([theta, 0, 0]))
        for j in range(1 + under):
            swap_xz = Pose(euler=[0, -np.pi / 2 + j * np.pi, 0])
            yield multiply(tool_pose, translate_rotate, swap_xz, translate_center, body_pose)


def convert_pose_z2x_y2z(pose):
    """for UR's "strange" TCP setup..."""
    point, quat = pose
    tform_mat = matrix_from_quat(quat)
    xaxis = tform_mat[:,0]
    yaxis = tform_mat[:,1]
    zaxis = tform_mat[:,2]
    swapped_mat = np.vstack([zaxis, np.cross(yaxis, zaxis), yaxis])
    return (point, quat_from_matrix(swapped_mat))


# @pytest.mark.wip
# def test_pickup_cylinder():
#     """TODO: this test_function can by pybullet-free"""
#     urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
#     srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')
#     urdf_pkg_name = 'ur_description'
#
#     ee_filename = compas_fab.get('universal_robot/ur_description/meshes/pychoreo_workshop_gripper/collision/pychoreo-workshop-gripper.stl')
#
#     # define TCP transformation
#     tcp_tf = Translation([0.2, 0, 0]) # in meters
#     pb_tool_from_tcp = pb_pose_from_Transformation(tcp_tf)
#     ee_mesh = Mesh.from_stl(ee_filename)
#
#     # create robot model in compas_fab
#     model = RobotModel.from_urdf_file(urdf_filename)
#     semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
#     robot = RobotClass(model, semantics=semantics)
#
#     base_link_name = robot.get_base_link_name()
#     ik_joint_names = robot.get_configurable_joint_names()
#     ik_tool_link_name = robot.get_end_effector_link_name()
#
#     # get disabled collisions
#     disabled_collisions = semantics.get_disabled_collisions()
#     assert len(disabled_collisions) == 10
#     assert ('base_link', 'shoulder_link') in disabled_collisions
#
#     # start pybullet env & convert compas_fab.robot to pybullet robot body
#     connect(use_gui=True)
#     pb_robot = create_pb_robot_from_ros_urdf(urdf_filename, urdf_pkg_name)
#     pb_ik_joints = joints_from_names(pb_robot, ik_joint_names)
#
#     # set start conf
#     ur5_start_conf = np.array([104., -80., -103., -86., 89., 194.]) / 180.0 * np.pi
#     set_joint_positions(pb_robot, pb_ik_joints, ur5_start_conf)
#
#     # add objects to pybullet scene & convert them into pybullet
#     with RosClient() as client:
#         assert client.is_connected, 'ros client not connected!'
#         robot.client = client
#
#         scene = PlanningScene(robot)
#         floor_mesh = Mesh.from_stl(compas_fab.get('planning_scene/floor.stl'))
#         floor_cm = CollisionMesh(floor_mesh, 'floor')
#         scene.add_collision_mesh(floor_cm)
#
#         cylinder_mesh = Mesh.from_stl(compas_fab.get('planning_scene/cylinder.stl'))
#         offset = Translation([0.4, 0, 0.001]) # in meters
#         mesh_transform(cylinder_mesh, offset)
#         cylinder_cm = CollisionMesh(cylinder_mesh, 'cylinder')
#         scene.add_collision_mesh(cylinder_cm)
#
#         time.sleep(1)
#
#         co_dict = client.get_collision_meshes_and_poses()
#         body_from_name = convert_meshes_and_poses_to_pybullet_bodies(co_dict)
#         floor_body = body_from_name['floor'][0]
#         cylinder_body = body_from_name['cylinder'][0]
#
#         # attach end effector mesh
#         # ikfast is built for base_link - ee_link, so here ee_link coincides with
#         # the tool mounting link
#         ee_bodies = attach_end_effector_geometry([ee_mesh], pb_robot, ik_tool_link_name)
#
#         if has_gui():
#             TCP_pb_pose = get_TCP_pose(pb_robot, ik_tool_link_name, tcp_tf, return_pb_pose=True)
#             handles = draw_pose(TCP_pb_pose, length=0.04)
#             print('initial env, press enter to continue')
#             # wait_for_user()
#             # for h in handles : remove_debug(h)
#
#             # grasp_gen = get_grasp_gen(pb_robot, 'side', ik_tool_link_name, pb_tool_from_tcp)
#             side_get_grasp_fn = lambda body: get_side_cylinder_grasps(body, under=True, \
#                                                              tool_pose=Pose(), \
#                                                              max_width=np.inf, grasp_length=0)
#             side_grasp_info = GraspInfo(side_get_grasp_fn, Pose(0.1*Point(z=-1)))
#
#             grasp_gen = get_grasp_gen(pb_robot, ik_tool_link_name, side_grasp_info)
#             body_pose = BodyPose(cylinder_body)
#             draw_pose(body_pose.pose, length=0.04)
#
#             ik_fn = ikfast_ur5.get_ik
#
#             # keep the world state
#             saved_world = WorldSaver()
#             # with LockRenderer():
#             command = plan_pickup_object(pb_robot, ik_tool_link_name, ik_joint_names, \
#                                          base_link_name, ik_fn, cylinder_body, grasp_gen, \
#                                          fixed=[], enable_self_collision=False,
#                                          extra_tcp_tf_fn=convert_pose_z2x_y2z)
#
#             assert command is None, 'Unable to find a plan!'
#
#             if has_gui():
#                 saved_world.restore()
#                 update_state()
#                 command.refine(num_steps=10).execute(time_step=0.002)
#
#         # clean up the planning scene in the ros backend
#         scene.remove_collision_mesh('floor')
#         scene.remove_collision_mesh('cylinder')
