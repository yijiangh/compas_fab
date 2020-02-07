"""
********************************************************************************
compas_fab.assembly
********************************************************************************

.. currentmodule:: compas_fab.assembly

This package contains data structures and utility functions for representing
discrete assembly objects and associated fabrication information.

Data structures
----------------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    Assembly
    Element
    VirtualJoint

Unit geometry containers
--------------------------
Classes used to specify a discrete object's mesh, intial/goal poses, and grasp information.

.. autosummary::
    :toctree: generated/
    :nosignatures:

    UnitGeometry
    Grasp

Network indexing utilities
--------------------------
Utility functions to help indexing `Element` and `VirtualJoint` in the `Assembly` network. These functions should be removed in future implementations (Gonzalo and I talked about this in the summer...)

.. autosummary::
    :toctree: generated/
    :nosignatures:

    element_vert_key
    extract_element_vert_id
    virtual_joint_key
    extract_virtual_joint_vert_id
    obj_name
    element_neighbor_to_joint

"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from .datastructures.assembly import *
from .datastructures.element import *
from .datastructures.virtual_joint import *
from .datastructures.unit_geometry import *
from .datastructures.grasp import *
from .datastructures.utils import *

__all__ = [name for name in dir() if not name.startswith('_')]
