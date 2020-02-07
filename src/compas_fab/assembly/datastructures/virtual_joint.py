from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

# import json

from compas.geometry import Frame
from compas.datastructures import Mesh

from .unit_geometry import UnitGeometry
from .utils import virtual_joint_key, extract_virtual_joint_vert_id

__all__ = ['VirtualJoint']


class VirtualJoint(object):
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

    def __init__(self, id, connected_element_ids=[], is_grounded=False, unit_geometries=[]):
        self._key = virtual_joint_key(id)
        self._connected_element_ids = connected_element_ids
        self._is_grounded = is_grounded
        self._unit_geometries = unit_geometries

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

    @property
    def unit_geometries(self):
        return self._unit_geometries

    @unit_geometries.setter
    def unit_geometries(self, ug_list):
        self._unit_geometries = ug_list

    # --------------
    # constructors
    # --------------
    def to_data(self, mesh_path=None):
        data = {}
        data['vj_id'] = self.key_id
        data['connected_element_ids'] = self.connected_element_ids
        data['is_grounded'] = self.is_grounded
        data['unit_geometries'] = [ug.to_data(mesh_path) for ug in self.unit_geometries]
        return data

    @classmethod
    def from_data(cls, data):
        return cls(data['vj_id'], data['connected_element_ids'], data['is_grounded'], \
            unit_geometries=[UnitGeometry.from_data(ug_data) for ug_data in data['unit_geometries']])

    # --------------
    # repr
    # --------------
    def __repr__(self):
       return 'VJ#{}: connected e: {}, #unit_geo: {}, is_grounded: {}'.format( \
           self.key_id, self.connected_element_ids, len(self.unit_geometries), self.is_grounded)

if __name__ == "__main__":
    pass
