""" file IO util functions for parsing problem instance / saving results

"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import json
from copy import copy, deepcopy

from conrob_pybullet import *
from compas_fab.assembly.datastructures import Assembly, UnitGeometry, Grasp

from compas.datastructures import Mesh
from compas.geometry import Frame, Transformation

def extract_file_name(str_key):
    key_sep = str_key.split('.')
    return key_sep[0]


def warning_print(msg):
    print('\x1b[6;30;43m' + msg + '\x1b[0m')


def load_assembly_package(package_json_path, scale=1.0):
    # os.path.join(instance_dir, instance_name, 'json', instance_name + '.json')
    with open(package_json_path, 'r') as f:
        json_data = json.loads(f.read())

    assembly = Assembly.from_package(json_data)
    for e_id in assembly.element_geometries.keys():
        assembly.element_geometries[e_id].rescale(scale)
    for so_id in assembly.static_obstacle_geometries.keys():
        assembly.static_obstacle_geometries[so_id].rescale(scale)

    static_obstacles = []
    for ug in assembly.static_obstacle_geometries.values():
        static_obstacles.extend(ug.mesh)

    return assembly.element_geometries, static_obstacles
