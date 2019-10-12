from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

# import json

from compas.geometry import Frame
from compas.datastructures import Mesh

from compas_fab.assembly.datastructures.utils import virtual_joint_key, extract_virtual_joint_vert_id

__all__ = ['VirtualJoint']

class VirtualJoint(object):

    __module__ = 'compas_fab.assembly.datastructures'

    def __init__(self, id, connected_element_ids=[], is_grounded=False):
        self._key = virtual_joint_key(id)
        self._connected_element_ids = connected_element_ids
        self._is_grounded = is_grounded

    @property
    def key(self):
        return self._key

    @property
    def key_id(self):
        return extract_virtual_joint_vert_id(self.key)

    @property
    def connected_element_ids(self):
        return self._connected_element_ids

    @property
    def is_grounded(self):
        return self._is_grounded

    @connected_element_ids.setter
    def connected_element_ids(self, e_ids):
        self._connected_element_ids = e_ids

    # --------------
    # constructors
    # --------------
    def to_data(self):
        data = {}
        data['vj_id'] = self.key_id
        data['connected_element_ids'] = self.connected_element_ids
        data['is_grounded'] = self.is_grounded
        return data

    @classmethod
    def from_data(cls, data):
        return cls(data['vj_id'], data['connected_element_ids'], data['is_grounded'])

if __name__ == "__main__":
    pass
