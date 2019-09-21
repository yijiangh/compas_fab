from conrob_pybullet import joints_from_names, link_from_name, set_joint_positions, \
    wait_for_duration, wait_for_user, create_attachment, set_pose, has_gui

def display_trajectory_chunk(robot, ik_joint_names,
                             trajectory, \
                             ee_attachs=[], grasped_attach=[],
                             time_step=0.1, step_sim=False, per_conf_step=False):
    # enable_gravity()
    ik_joints = joints_from_names(robot, ik_joint_names)

    for conf in trajectory['points']:
        set_joint_positions(robot, ik_joints, conf['values'])

        for ea in ee_attachs: ea.assign()
        for at in grasped_attach: at.assign()

        if not per_conf_step:
            wait_for_duration(time_step)
        else:
            wait_for_user()

    if step_sim: wait_for_user()

def display_picknplace_trajectories(robot, ik_joint_names, ee_link_name,
                                    unit_geos, trajectories, \
                                    element_seq=[],
                                    from_seq_id=0, to_seq_id=None,
                                    ee_attachs=[],
                                    cartesian_time_step=0.075,
                                    transition_time_step=0.1, step_sim=False, per_conf_step=False):
    # enable_gravity()
    ik_joints = joints_from_names(robot, ik_joint_names)
    end_effector_link = link_from_name(robot, ee_link_name)
    element_seq = element_seq or list(len(unit_geos))

    from_seq_id = from_seq_id or 0
    to_seq_id = to_seq_id or len(element_seq)-1
    assert 0 <= from_seq_id and from_seq_id < len(element_seq)
    assert from_seq_id <= to_seq_id and to_seq_id < len(element_seq)

    for seq_id in range(0, from_seq_id):
        e_id = element_seq[seq_id]
        for e_body in unit_geos[e_id].pybullet_bodies:
            set_pose(e_body, unit_geos[e_id].goal_pb_pose)

    assert has_gui()
    set_joint_positions(robot, ik_joints, trajectories[0]['place2pick']['points'][0]['values'])
    for ea in ee_attachs: ea.assign()
    # wait_for_user()

    for seq_id in range(from_seq_id, to_seq_id + 1):
        unit_geo = unit_geos[element_seq[seq_id]]
        if seq_id-from_seq_id > len(trajectories):
            break

        unit_picknplace = trajectories[seq_id - from_seq_id]

        print('seq #{} : place 2 pick tranisiton'.format(seq_id))
        if 'place2pick' in unit_picknplace and unit_picknplace['place2pick']['points']:
            # place2pick transition
            for conf in unit_picknplace['place2pick']['points']:
                set_joint_positions(robot, ik_joints, conf['values'])
                for ea in ee_attachs: ea.assign()
                if not per_conf_step:
                    wait_for_duration(transition_time_step)
                else:
                    wait_for_user()
        else:
            print('seq #{} does not have place to pick transition plan found!'.format(seq_id))

        if step_sim: wait_for_user()

        print('seq #{} : pick approach'.format(seq_id))
        # pick_approach
        # unit_picknplace['pick_approach'].pop(0)
        for conf in unit_picknplace['pick_approach']['points']:
            set_joint_positions(robot, ik_joints, conf['values'])
            for ea in ee_attachs: ea.assign()
            if not per_conf_step:
                wait_for_duration(cartesian_time_step)
            else:
                wait_for_user()

        if step_sim: wait_for_user()

        # pick attach
        attachs = []
        for e_body in unit_geo.pybullet_bodies:
            attachs.append(create_attachment(robot, end_effector_link, e_body))
        # add_fixed_constraint(brick.body, robot, end_effector_link)

        print('seq #{} : pick retreat'.format(seq_id))
        # pick_retreat
        for conf in unit_picknplace['pick_retreat']['points']:
            set_joint_positions(robot, ik_joints, conf['values'])
            for ea in ee_attachs: ea.assign()
            for at in attachs: at.assign()
            if not per_conf_step:
                wait_for_duration(cartesian_time_step)
            else:
                wait_for_user()

        if step_sim: wait_for_user()

        print('seq #{} : pick 2 place tranisiton'.format(seq_id))
        # pick2place transition
        if 'pick2place' in unit_picknplace and unit_picknplace['pick2place']['points']:
            for conf in unit_picknplace['pick2place']['points']:
                set_joint_positions(robot, ik_joints, conf['values'])
                for ea in ee_attachs: ea.assign()
                for at in attachs: at.assign()
                if not per_conf_step:
                    wait_for_duration(transition_time_step)
                else:
                    wait_for_user()
        else:
            print('seq #{} does not have pick to place transition plan found!'.format(seq_id))

        if step_sim: wait_for_user()

        print('seq #{} : place approach'.format(seq_id))
        # place_approach
        for conf in unit_picknplace['place_approach']['points']:
            set_joint_positions(robot, ik_joints, conf['values'])
            for ea in ee_attachs: ea.assign()
            for at in attachs: at.assign()
            if not per_conf_step:
                wait_for_duration(cartesian_time_step)
            else:
                wait_for_user()

        if step_sim: wait_for_user()

        # place detach
        # remove_fixed_constraint(brick.body, robot, end_effector_link)

        print('seq #{} : place retreat'.format(seq_id))
        # place_retreat
        for conf in unit_picknplace['place_retreat']['points']:
            set_joint_positions(robot, ik_joints, conf['values'])
            for ea in ee_attachs: ea.assign()
            if not per_conf_step:
                wait_for_duration(cartesian_time_step)
            else:
                wait_for_user()

        if step_sim: wait_for_user()

        if seq_id == to_seq_id or seq_id-from_seq_id == len(trajectories) - 1:
            if 'return2idle' in unit_picknplace:
                for conf in unit_picknplace['return2idle']['points']:
                    set_joint_positions(robot, ik_joints, conf['values'])
                    for ea in ee_attachs: ea.assign()
                    if not per_conf_step:
                        wait_for_duration(transition_time_step)
                    else:
                        wait_for_user()
