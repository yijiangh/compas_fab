"""
*******************************************************************************
compas_fab.assembly.datastructures
*******************************************************************************

.. module:: compas_fab.assembly.datastructures

This package contains data structures for representing discrete assembly objects and associated fabrication information.

"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from .assembly import *
from .element import *
from .virtual_joint import *
from .unit_geometry import *
from .grasp import *
from .utils import *

__all__ = [name for name in dir() if not name.startswith('_')]
