from __future__ import print_function
import os
import random
import time
import pytest
import numpy as np
from numpy.testing import assert_array_almost_equal

from compas.geometry import Frame
from compas.geometry import Translation
from compas.datastructures import Mesh, mesh_transform
from compas.robots import RobotModel

import compas_fab
from compas_fab.backends import RosClient
from compas_fab.robots import Robot as RobotClass
from compas_fab.robots import RobotSemantics
from compas_fab.robots import PlanningScene
from compas_fab.robots import CollisionMesh
from compas_fab.robots.ur5 import Robot

from compas_fab.backends.pybullet import attach_end_effector_geometry, \
convert_mesh_to_pybullet_body, get_TCP_pose, create_pb_robot_from_ros_urdf, \
convert_meshes_and_poses_to_pybullet_bodies, pb_pose_from_Transformation
from compas_fab.backends.pybullet import get_grasp_gen

# from conrob_pybullet import load_pybullet, connect, disconnect, wait_for_user, \
#     LockRenderer, has_gui, get_model_info, get_pose, euler_from_quat, draw_pose, \
#     get_link_pose, link_from_name, create_attachment, add_fixed_constraint,\
#     create_obj, set_pose

from conrob_pybullet import load_pybullet, connect, disconnect, wait_for_user, \
    LockRenderer, has_gui, get_model_info, get_pose, euler_from_quat, draw_pose, \
    get_link_pose, link_from_name, create_attachment, add_fixed_constraint, \
    create_obj, set_pose, get_sample_fn, violates_limits, joints_from_names, \
    set_joint_positions, remove_debug, get_joint_limits, WorldSaver, \
    LockRenderer, update_state, end_effector_from_body, approach_from_grasp, \
    unit_pose, approximate_as_prism, point_from_pose, multiply, quat_from_euler, \
    approximate_as_cylinder

from conrob_pybullet import Pose, Point, BodyPose, GraspInfo


def test_convert_compas_robot_to_pybullet_robot():
    # get ur robot model from local test data that's shipped with compas_fab
    # does not need client connection here

    urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
    srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')
    ee_filename = compas_fab.get('universal_robot/ur_description/meshes/pychoreo_workshop_gripper/collision/pychoreo-workshop-gripper.stl')
    urdf_pkg_name = 'ur_description'

    # geometry file is not loaded here
    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
    robot = RobotClass(model, semantics=semantics)
    ee_link_name = robot.get_end_effector_link_name()
    # print('ee link: {}'.format(ee_link_name))

    # parse end effector mesh
    ee_mesh = Mesh.from_stl(ee_filename)

    # define TCP transformation
    tcp_tf = Translation([0.2, 0, 0]) # in meters

    connect(use_gui=False)
    # if the following returns without error, we are good
    pb_robot = create_pb_robot_from_ros_urdf(urdf_filename, urdf_pkg_name)

    # get disabled collisions
    disabled_collisions = semantics.get_disabled_collisions()
    assert len(disabled_collisions) == 10
    assert ('base_link', 'shoulder_link') in disabled_collisions

    # attach tool
    ee_bodies = attach_end_effector_geometry([ee_mesh], pb_robot, ee_link_name)

    # draw TCP frame in pybullet
    TCP_pb_pose = get_TCP_pose(pb_robot, ee_link_name, tcp_tf, return_pb_pose=True)

    if has_gui():
        draw_pose(TCP_pb_pose, length=0.04)
        wait_for_user()


def test_convert_planning_scene_collision_objects_to_pybullet_obstacles():
    with RosClient() as client:
        assert client.is_connected
        robot = Robot(client)

        scene = PlanningScene(robot)
        mesh = Mesh.from_stl(compas_fab.get('planning_scene/floor.stl'))
        cm = CollisionMesh(mesh, 'floor')
        scene.add_collision_mesh(cm)

        # See: https://github.com/compas-dev/compas_fab/issues/63#issuecomment-519525879
        time.sleep(1)

        connect(use_gui=False)
        co_dict = client.get_collision_meshes_and_poses()
        body_dict = convert_meshes_and_poses_to_pybullet_bodies(co_dict)

        assert 'floor' in body_dict
        pyb_pose = get_pose(body_dict['floor'][0])
        input_frame = list(co_dict.values())[0]['mesh_poses'][0]
        assert pyb_pose[0] == input_frame.point
        assert_array_almost_equal(euler_from_quat(pyb_pose[1]), input_frame.euler_angles())
        if has_gui():
            wait_for_user()

        scene.remove_collision_mesh('floor')


def get_side_grasps(body, under=False, tool_pose=Pose(), body_pose=unit_pose(),
                    max_width=np.inf, grasp_length=0, top_offset=0.03):
    center, (w, l, h) = approximate_as_prism(body, body_pose=body_pose)
    translate_center = Pose(point=point_from_pose(body_pose)-center)
    grasps = []
    x_offset = 0
    # x_offset = h/2 - top_offset
    for j in range(1 + under):
        swap_xz = Pose(euler=[0, -np.pi / 2 + j * np.pi, 0])
        # swap_xz = Pose()
        if w <= max_width:
            translate_z = Pose(point=[x_offset, 0, l / 2 - grasp_length])
            for i in range(2):
                rotate_z = Pose(euler=[np.pi / 2 + i * np.pi, 0, 0])
                grasps += [multiply(tool_pose, translate_z, rotate_z, #swap_xz,
                                    translate_center, body_pose)]  # , np.array([w])
        if l <= max_width:
            translate_z = Pose(point=[x_offset, 0, w / 2 - grasp_length])
            for i in range(2):
                rotate_z = Pose(euler=[i * np.pi, 0, 0])
                grasps += [multiply(tool_pose, translate_z, rotate_z, #swap_xz,
                                    translate_center, body_pose)]  # , np.array([l])
    return grasps


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


@pytest.mark.wip
def test_grasp_generator():
    urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
    srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')
    urdf_pkg_name = 'ur_description'

    ee_filename = compas_fab.get('universal_robot/ur_description/meshes/pychoreo_workshop_gripper/collision/pychoreo-workshop-gripper.stl')

    # define TCP transformation
    tcp_tf = Translation([0.2, 0, 0]) # in meters
    pb_tool_from_tcp = pb_pose_from_Transformation(tcp_tf)
    ee_mesh = Mesh.from_stl(ee_filename)

    # create robot model in compas_fab
    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
    robot = RobotClass(model, semantics=semantics)

    base_link_name = robot.get_base_link_name()
    ik_joint_names = robot.get_configurable_joint_names()
    ik_tool_link_name = robot.get_end_effector_link_name()

    # start pybullet env & convert compas_fab.robot to pybullet robot body
    connect(use_gui=True)
    pb_robot = create_pb_robot_from_ros_urdf(urdf_filename, urdf_pkg_name)
    pb_ik_joints = joints_from_names(pb_robot, ik_joint_names)

    # set start conf
    ur5_start_conf = np.array([104., -80., -103., -86., 89., 194.]) / 180.0 * np.pi
    set_joint_positions(pb_robot, pb_ik_joints, ur5_start_conf)

    # add objects to pybullet scene & convert them into pybullet
    with RosClient() as client:
        assert client.is_connected, 'ros client not connected!'
        robot.client = client

        scene = PlanningScene(robot)
        floor_mesh = Mesh.from_stl(compas_fab.get('planning_scene/floor.stl'))
        floor_cm = CollisionMesh(floor_mesh, 'floor')
        scene.add_collision_mesh(floor_cm)

        cylinder_mesh = Mesh.from_stl(compas_fab.get('planning_scene/cylinder.stl'))
        offset = Translation([0.4, 0, 0.001]) # in meters
        mesh_transform(cylinder_mesh, offset)
        cylinder_cm = CollisionMesh(cylinder_mesh, 'cylinder')
        scene.add_collision_mesh(cylinder_cm)

        time.sleep(1)

        co_dict = client.get_collision_meshes_and_poses()
        body_from_name = convert_meshes_and_poses_to_pybullet_bodies(co_dict)
        floor_body = body_from_name['floor'][0]
        cylinder_body = body_from_name['cylinder'][0]

        # attach end effector mesh
        # ikfast is built for base_link - ee_link, so here ee_link coincides with
        # the tool mounting link
        ee_bodies = attach_end_effector_geometry([ee_mesh], pb_robot, ik_tool_link_name)

        TCP_pb_pose = get_TCP_pose(pb_robot, ik_tool_link_name, tcp_tf, return_pb_pose=True)

        if has_gui():
            handles = draw_pose(TCP_pb_pose, length=0.04)
            print('initial env, press enter to continue')

        # grasp_gen = get_grasp_gen(pb_robot, 'side', ik_tool_link_name, pb_tool_from_tcp)
        side_get_grasp_fn = lambda body: get_side_cylinder_grasps(body, under=True, \
                                                         tool_pose=Pose(), \
                                                         max_width=np.inf, grasp_length=0)
        side_grasp_info = GraspInfo(side_get_grasp_fn, Pose(0.1*Point(x=-1)))

        grasp_gen = get_grasp_gen(pb_robot, ik_tool_link_name, side_grasp_info)
        body_pose = BodyPose(cylinder_body)
        if has_gui():
            draw_pose(body_pose.pose, length=0.04)

        for body_grasp, in grasp_gen(cylinder_body):
            world_from_gripper = end_effector_from_body(body_pose.pose, \
                                                        body_grasp.grasp_pose)
            world_from_approach = approach_from_grasp(body_grasp.approach_pose, \
                                                world_from_gripper)
            if has_gui():
                draw_pose(world_from_gripper, length=0.04)
                draw_pose(world_from_approach, length=0.02)
                wait_for_user()

        scene.remove_collision_mesh('floor')
        scene.remove_collision_mesh('cylinder')
