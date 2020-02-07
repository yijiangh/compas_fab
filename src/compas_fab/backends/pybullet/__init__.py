
"""
*******************************************************************************
compas_fab.backends.pybullet
*******************************************************************************

.. module:: compas_fab.backends.pybullet

Package with functionality to interact with `pybullet <https://pybullet.org/wordpress//>`_ and `pybullet_planning <https://pybullet-planning.readthedocs.io/>`.

Warning: we might only support py3.2 because of the use of tempfile.TemporaryDirectory.

Mesh to Body conversion
========================

.. autosummary::
    :toctree: generated/
    :nosignatures:

    convert_mesh_to_pybullet_body
    convert_meshes_and_poses_to_pybullet_bodies

Frame to Pose conversion
========================
In pybullet_planning, poses are represented by (point, quat).

.. autosummary::
    :toctree: generated/
    :nosignatures:

    pb_pose_from_Transformation
    pb_pose_from_Frame
    Frame_from_pb_pose
    Frame_from_pos_rot

Utility functions
=================
Utility functions for routine operations, e.g. attach end effector in pybullet, getting current TCP pose, etc. These functions can be removed in the future, since they are here just for my own convenience :)

.. autosummary::
    :toctree: generated/
    :nosignatures:

    attach_end_effector_geometry
    get_TCP_pose

"""

from __future__ import absolute_import
import compas

from .body import *
from .pose import *
from .robot import *

__all__ = [name for name in dir() if not name.startswith('_')]
