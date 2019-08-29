from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import cProfile
import pstats
import sys
import time

from choreo.csp import backtracking_search
from choreo.assembly_csp import AssemblyCSP
from choreo.assembly_csp import next_variable_in_sequence, random_value_ordering, \
    cmaps_forward_check, traversal_to_ground_value_ordering, cmaps_value_ordering
from choreo.choreo_utils import set_cmaps_using_seq

from conrob_pybullet import WorldSaver

LOG_CSP = True
LOG_CSP_PATH = None

def plan_sequence(robot, obstacles, assembly_network,
                  problem_type='picknplace'
                  search_method='forward', value_ordering_method='random', use_layer=False, file_name=None):
    """[summary]

    Parameters
    ----------
    robot : [type]
        [description]
    obstacles : [type]
        [description]
    assembly_network : compas_fab.assembly.datastructures.Assembly
        [description]
    stiffness_checker : [type], optional
        [description], by default None
    search_method : str, optional
        [description], by default 'forward'
    value_ordering_method : str, optional
        [description], by default 'random'
    use_layer : bool, optional
        [description], by default False
    file_name : [type], optional
        [description], by default None

    Returns
    -------
    [type]
        [description]
    """
    pr = cProfile.Profile()
    pr.enable()

    # generate AssemblyCSP problem
    print('search method: {0},\nvalue ordering method: {1},\nuse_layer: {2}'.format(
        search_method, value_ordering_method, use_layer))

    saved_world = WorldSaver()

    csp = AssemblyCSP(robot, obstacles, assembly_network=assembly_network,
                      search_method=search_method,
                      vom=value_ordering_method,
                      use_layer=use_layer)
    csp.logging = LOG_CSP

    try:
        if search_method == 'forward':
            if value_ordering_method == 'random':
                seq, csp = backtracking_search(csp, select_unassigned_variable=next_variable_in_sequence,
                                               order_domain_values=random_value_ordering,
                                               inference=cmaps_forward_check)
            else:
                seq, csp = backtracking_search(csp, select_unassigned_variable=next_variable_in_sequence,
                                               order_domain_values=cmaps_value_ordering,
                                               inference=cmaps_forward_check)
        # elif search_method == 'backward':
        #     if value_ordering_method == 'random':
        #         seq, csp = backtracking_search(csp, select_unassigned_variable=next_variable_in_sequence,
        #                                        order_domain_values=random_value_ordering)
        #     else:
        #         seq, csp = backtracking_search(csp, select_unassigned_variable=next_variable_in_sequence,
        #                                        order_domain_values=traversal_to_ground_value_ordering)
    except KeyboardInterrupt:
        if csp.logging and file_name:
            csp.write_csp_log(file_name, log_path=LOG_CSP_PATH)

        pr.disable()
        pstats.Stats(pr).sort_stats('tottime').print_stats(10)
        sys.exit()

    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)

    print('# of assigns: {0}'.format(csp.nassigns))
    print('# of bt: {0}'.format(csp.nbacktrackings))
    print('constr check time: {0}'.format(csp.constr_check_time))
    # print('final seq: {}'.format(seq))

    if search_method == 'backward':
        order_keys = list(seq.keys())
        order_keys.reverse()
        rev_seq = {}
        for i in seq.keys():
            rev_seq[order_keys[i]] = seq[i]
        seq = rev_seq
        st_time = time.time()
        csp = set_cmaps_using_seq(rev_seq, csp)
        print('pruning time: {0}'.format(time.time() - st_time))

    seq_poses = {}
    for i in sorted(seq.keys()):
        e_id = seq[i]
        # feasible ee directions
        seq_poses[i] = []
        assert(e_id in csp.cmaps)
        for cmap_id, free_flag in enumerate(csp.cmaps[e_id]):
            if free_flag:
                # phi, theta = cmap_id2angle(cmap_id)
                # TODO: convert cmap_id to EE pose, get from UnitGeometry?
                seq_poses[i].append(EEDirection(phi=phi, theta=theta))

    if csp.logging and file_name:
        csp.write_csp_log(file_name, log_path=LOG_CSP_PATH)

    saved_world.restore()

    return seq, seq_poses
