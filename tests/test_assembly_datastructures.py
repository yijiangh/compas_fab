from __future__ import print_function
import os
import time
import json
from collections import OrderedDict
import pytest

import math
import numpy as np
from numpy.testing import assert_array_almost_equal

from compas.geometry import Frame, Transformation
from compas_fab.assembly.datastructures import Assembly, UnitGeometry, Grasp


@pytest.mark.assembly_ds
def test_serialization():
    # Grasp
    grasp = Grasp.from_frames(1, 1, Frame.worldXY(), Frame.worldYZ(), Frame.worldZX())
    other = Grasp.from_data(grasp.to_data())
    # TODO: check equality
