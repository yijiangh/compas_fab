from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

import json

from compas.geometry import Frame, Transformation
from compas_fab.assembly.datastructures.utils import element_vert_key, extract_element_vert_id

__all__ = ['Element']

class Element(object):

    __module__ = 'compas_fab.assembly.datastructures'

    def __init__(self, id, layer_id=0, is_grounded=False):
        super(Element, self).__init__()
        self._key = element_vert_key(id)
        self._unit_geometry = None

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
    def unit_geometry(self):
        return self._unit_geometry

    @unit_geometry.setter
    def unit_geometry(self, ug):
        self._unit_geometry = ug

    # TODO: include unit geo
    def to_data(self):
        data = {}
        data['e_id'] = self.key_id
        data['layer_id'] = self.layer_id
        data['is_grounded'] = self.is_grounded
        return data

    @classmethod
    def from_data(cls, data):
        return cls(data['e_id'], layer_id=data['layer_id'], is_grounded=data['is_grounded'])

# ==============================================================================
# Main
# ==============================================================================
if __name__ == "__main__":
    pass
