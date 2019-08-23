from __future__ import print_function
import os
import time
import pytest
import math

import numpy as np
from numpy.testing import assert_equal, assert_almost_equal

from compas.datastructures import Mesh
from compas.geometry import Frame

import compas_fab
from compas_fab.robots import Robot as RobotClass
from compas_fab.robots.ur5 import Robot
from compas_fab.robots import CollisionMesh
from compas_fab.backends import RosClient
from compas_fab.robots import PlanningScene
from compas_fab.robots import Configuration

@pytest.mark.ros_srv
def test_get_collision_objects_from_planning_scene():
    with RosClient() as client:
        assert client.is_connected
        robot = Robot(client)

        scene = PlanningScene(robot)
        scene.remove_all_collision_objects()
        mesh = Mesh.from_stl(compas_fab.get('planning_scene/floor.stl'))
        cm = CollisionMesh(mesh, 'floor')
        scene.add_collision_mesh(cm)

        # See: https://github.com/compas-dev/compas_fab/issues/63#issuecomment-519525879
        time.sleep(0.5)

        co_dict = client.get_collision_meshes_and_poses()
        assert len(co_dict) == 1 and 'floor' in co_dict
        assert len(co_dict['floor']['meshes']) == 1
        assert len(co_dict['floor']['mesh_poses']) == 1
        floor_mesh = co_dict['floor']['meshes'][0]
        assert floor_mesh.number_of_vertices() == 4
        assert floor_mesh.number_of_faces() == 2
        assert floor_mesh.number_of_edges() == 5

        floor_frame = co_dict['floor']['mesh_poses'][0]
        assert floor_frame == Frame.worldXY()


# @pytest.mark.ros_srv
def test_single_query_motion_plan_from_ee_frame():
    with RosClient() as client:
        robot = Robot(client)

        frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
        tolerance_position = 0.001
        tolerance_axes = [math.radians(1)] * 3

        start_configuration = Configuration.from_revolute_values([-0.042, 4.295, 0, -3.327, 4.755, 0.])
        group = robot.main_group_name

        # create goal constraints from frame
        goal_constraints = robot.constraints_from_frame(frame,
                                                        tolerance_position,
                                                        tolerance_axes,
                                                        group)

        trajectory = robot.plan_motion(goal_constraints,
                                       start_configuration,
                                       group,
                                       planner_id='RRT')

        print("Computed kinematic path with %d configurations." % len(trajectory.points))
        print("Executing this path at full speed would take approx. %.3f seconds." % trajectory.time_from_start)



@pytest.mark.ros_srv
def test_single_query_motion_plan_from_joint_conf():
    with RosClient() as client:
        robot = Robot(client)
        group = robot.main_group_name
        joint_names = robot.get_configurable_joint_names()

        scene = PlanningScene(robot)
        scene.remove_all_collision_objects()
        print('existing collision objs: {}'.format(scene.get_collision_meshes_and_poses()))

        current_jt_val = scene.get_joint_state().values()
        print(current_jt_val)
        # or current_jt_state = client.get_joint_state()
        ur5_idle_conf = [1.8151424220741026, -1.3962634015954636, -1.7976891295541593,
                        -1.5009831567151235, 1.5533430342749532, 3.385938748868999]

        # valid st and goal conf
        # st_conf_val = current_jt_val
        # goal_conf_val = ur5_idle_conf

        # valid st, but 'out of range' goal conf
        st_conf_val = ur5_idle_conf

        # this conf has out-of-limit (-pi ~ pi) `elbow_joint` value
        invalid_goal_conf_val = [3.3692990008117247, 4.006257566491165, 4.70930506391325,
                                 3.8508079839547396, 3.369299000811508, 3.0415926535897873]
        goal_conf_val = [6.134712063782542, 5.418520394278199, 1.573880243266336,
                         5.573969976814626, 6.134712063782833, 3.1415926535897922]

        st_conf = Configuration.from_revolute_values(st_conf_val)
        goal_conf = Configuration.from_revolute_values(goal_conf_val)

        print('is_valid? st_conf : {}'.format(
            client.is_joint_state_colliding(group, joint_names, st_conf.values)))
        print('is_valid? goal_conf : {}'.format(
            client.is_joint_state_colliding(group, joint_names, goal_conf.values)))

        # create goal constraints from joint conf
        goal_constraints = robot.constraints_from_configuration(goal_conf, [math.radians(5)] * 6, group)
        traj = robot.plan_motion(goal_constraints, st_conf, group, planner_id='RRTConnect',
                                 num_planning_attempts=20, allowed_planning_time=10)

        print("Motion plan from joint: computed kinematic path with %d configurations." % len(traj.points))
        print("Executing this path at full speed would take approx. %.3f seconds." % traj.time_from_start)


@pytest.mark.ros_srv
def test_is_joint_state_colliding():
    with RosClient() as client:
        robot = Robot(client)
        group = robot.main_group_name
        joint_names = robot.get_configurable_joint_names()

        idle_conf_val = [1.8151424220741026, -1.3962634015954636, -1.7976891295541593,
                       -1.5009831567151235, 1.5533430342749532, 3.385938748868999]
        self_collision_conf_val = [0.05092512883900069, 4.407050465744106, 3.4727222613517697,
                                   1.4450052336734978, 4.661463851545683, 0.0]

        # is_joint_state_colliding is not checking joint limits, so this one will pass
        # although the `elbow_joint` is over-limit
        violate_joint_limit = [3.3692990008117247, 4.006257566491165, 4.70930506391325,
                               3.8508079839547396, 3.369299000811508, 3.0415926535897873]

        choreo_example_sol = [6.134712063782542, 5.418520394278199, 1.573880243266336,
                         5.573969976814626, 6.134712063782833, 3.1415926535897922]

        assert not client.is_joint_state_colliding(group, joint_names, idle_conf_val)
        assert client.is_joint_state_colliding(group, joint_names, self_collision_conf_val)
        assert not client.is_joint_state_colliding(group, joint_names, violate_joint_limit)
        assert not client.is_joint_state_colliding(group, joint_names, choreo_example_sol)


@pytest.mark.ros_srv
@pytest.mark.js
def test_set_and_get_joint_state():
    with RosClient() as client:
        robot = Robot(client)
        group = robot.main_group_name
        joint_names = robot.get_configurable_joint_names()

        # the following conf has `elbow_joint` (2) out of limit
        # given_sol = [3.3692990008117247, 4.006257566491165, 4.70930506391325,
        #              3.8508079839547396, 3.369299000811508, 3.0415926535897873]
        # given_sol = [0] * 6
        given_sol = [6.134712063782542, 5.418520394278199, 1.573880243266336, 5.573969976814626, 6.134712063782833, 3.1415926535897922]

        client.set_joint_positions(group, joint_names, given_sol)
        current_jt_val = client.get_joint_state().values()
        assert_equal(given_sol, list(current_jt_val))
