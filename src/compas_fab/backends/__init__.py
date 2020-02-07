"""
********************************************************************************
compas_fab.backends
********************************************************************************

.. currentmodule:: compas_fab.backends

This package contains classes backends for simulation, planning and execution.

V-REP
-----

.. autosummary::
    :toctree: generated/
    :nosignatures:

    VrepClient

ROS
---

.. autosummary::
    :toctree: generated/
    :nosignatures:

    RosClient
    RosFileServerLoader

Pybullet
--------

This module currently contains only the type conversion utility function to make compas types compatible with `pybullet_planning <https://pybullet-planning.readthedocs.io/>`_. Note that this module imports `pybullet_planning`, which does not runs on IronPython. Thus, though violating PEP guidelines for importing modules, pybullet_planning functions are imported at runtime (within functions).

.. autosummary::
    :toctree: generated/
    :nosignatures:

    convert_mesh_to_pybullet_body
    convert_meshes_and_poses_to_pybullet_bodies
    pb_pose_from_Transformation
    pb_pose_from_Frame
    Frame_from_pb_pose
    Frame_from_pos_rot

Long-running tasks
------------------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    FutureResult
    CancellableFutureResult

Exceptions
----------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    BackendError
    RosError
    RosValidationError
    VrepError

"""

from .exceptions import *               # noqa: F401,F403
from .tasks import *                    # noqa: F401,F403
from .ros.client import *               # noqa: F401,F403
from .ros.exceptions import *           # noqa: F401,F403
from .ros.fileserver_loader import *    # noqa: F401,F403
from .vrep.client import *              # noqa: F401,F403
from .pybullet import *

__all__ = [name for name in dir() if not name.startswith('_')]
