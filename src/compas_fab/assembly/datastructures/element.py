from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

import json

from compas.geometry import Frame, Transformation
from .unit_geometry import UnitGeometry
from .utils import element_vert_key, extract_element_vert_id

__all__ = ['Element']


class Element(object):
    """todo

    Parameters
    ----------
    object : [type]
        [description]

    Returns
    -------
    [type]
        [description]
    """

    __module__ = 'compas_fab.assembly.datastructures'

    def __init__(self, id, layer_id=0, is_grounded=False, unit_geometries=None):
        self._key = element_vert_key(id)
        self._unit_geometries = unit_geometries or []
        self._layer_id = layer_id
        self.is_grounded = is_grounded
        self.to_ground_dist = 1e10

    @property
    def key(self):
        return self._key

    @property
    def key_id(self):
        return extract_element_vert_id(self.key)

    @property
    def layer_id(self):
        return self._layer_id

    @property
    def unit_geometries(self):
        return self._unit_geometries

    @unit_geometries.setter
    def unit_geometries(self, ug_list):
        self._unit_geometries = ug_list

    # TODO: include unit geo
    def to_data(self, mesh_path=None):
        data = {}
        data['e_id'] = self.key_id
        data['layer_id'] = self.layer_id
        data['is_grounded'] = self.is_grounded
        data['unit_geometries'] = [ug.to_data(mesh_path) for ug in self.unit_geometries]
        return data

    @classmethod
    def from_data(cls, data):
        return cls(data['e_id'], layer_id=data['layer_id'], is_grounded=data['is_grounded'],
            unit_geometries=[UnitGeometry.from_data(ug_data) for ug_data in data['unit_geometries']])

    # --------------
    # repr
    # --------------
    def __repr__(self):
       return 'E#{}: #unit_geo: {}, is_grounded: {}, layer_id: {}'.format( \
           self.key_id, len(self.unit_geometries), self.is_grounded, self.layer_id)

# ==============================================================================
# Main
# ==============================================================================
if __name__ == "__main__":
    pass
