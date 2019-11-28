from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

import os
import json
from collections import OrderedDict

from compas.datastructures.network import Network
from compas.geometry import Frame, cross_vectors, Transformation

from .element import Element
from .unit_geometry import UnitGeometry
from .virtual_joint import VirtualJoint
from .utils import transform_cmesh, element_vert_key, \
    virtual_joint_key, extract_element_vert_id, obj_name, STATIC_OBSTACLE_PREFIX, extract_virtual_joint_vert_id

__all__ = ['Assembly']

# the default description frame
WORLD_FRAME = Frame.worldXY()

class Assembly(object):
    """A data structure for discrete element assemblies.
    The users only work with integer indices,
    all the keys are kept internally.
    id means int index.
    key means str.
    An assembly is composed of:
        - a collection of discrete element geometries
        - a collection of physical joint geometries (optional)
        - a network modeling connectivity and interface info between elements and joints
    There are two types of vertices in this network:
        - element_vert
            represents unit assembly element
        - virtual_joint_vert
            represents a connection between elements. But it does not necessarily
            refer to a phyiscal joint. For example in the case of compression-only
            blocks, the virtual joint models contact information between two blocks,
            where no real joint exists between these two blocks.

    TODO: safeguarding missing key?
          response if init vj before elements?
    """

    __module__ = 'compas_assembly.datastructures'

    def __init__(self, elements=None, attributes=None):
        super(Assembly, self).__init__()
        self._net = Network()
        self._static_obstacle_geometries = {}
        self._num_of_elements = 0
        self._num_of_virtual_joints = 0

        # grouping elements
        # TODO: virtual joints can be included as well
        self._layer_element_ids = {}

    @property
    def network(self):
        return self._net

    # --------------
    # add / get functions for element(s), virtual joint(s)
    # --------------

    def add_element(self, element_instance):
        """[summary]
        Parameters
        ----------
        element_instance : compas.assembly.datastructures.Element
            [description]
        """
        # unit geometry's object frame transformed to origin
        self._net.add_vertex(key=element_instance.key, element=element_instance, tag='element')

        if element_instance.layer_id in self._layer_element_ids.keys():
            self._layer_element_ids[element_instance.layer_id].append(element_instance.key)
        else:
            self._layer_element_ids[element_instance.layer_id] = [element_instance.key]

        self._num_of_elements += 1

    def get_element(self, e_id):
        if isinstance(e_id, int):
            e_key = element_vert_key(e_id)
        elif isinstance(e_id, str) and extract_element_vert_id(e_id) is not None:
            e_key = e_id
        else:
            return None
        # assert(element_vert_key(e_id) in self._net.vertex)
        return self._net.get_vertex_attribute(e_key, 'element')

    @property
    def elements(self):
        """User should use int-indexed dict
        Parameters
        ----------
        key_indexed : bool, optional
            [description], by default False
        Returns
        -------
        [type]
            [description]
        """
        e_dict = dict()
        for v_key in self._net.vertex.keys():
            if self.get_element(v_key):
                e_dict[extract_element_vert_id(v_key)] = self.get_element(v_key)
        return e_dict

    def add_virtual_joint(self, vj_instance):
        """
        Network edges are added here.
        Parameters
        ----------
        vj_instance : compas.assembly.datastructures.VirtualJoint
            [description]
        """
        # TODO: assign virtual id index here
        self._net.add_vertex(key=vj_instance.key, virtual_joint=vj_instance, tag='virtual_joint')
        for e_id in vj_instance.connected_element_ids:
            # assert(element_vert_key(e_id) in self._net.vertex)
            self._net.add_edge(element_vert_key(e_id), vj_instance.key)
        self._num_of_virtual_joints += 1

    def get_virtual_joint(self, vj_id):
        if isinstance(vj_id, int):
            vj_key = virtual_joint_key(vj_id)
        elif isinstance(vj_id, str) and extract_virtual_joint_vert_id(vj_id) is not None:
            vj_key = vj_id
        else:
            return None
        return self._net.get_vertex_attribute(vj_key, 'virtual_joint')

    @property
    def virtual_joints(self):
        vj_dict = dict()
        for v_key in self._net.vertex.keys():
            if self.get_virtual_joint(v_key):
                vj_dict[extract_virtual_joint_vert_id(v_key)] = self.get_virtual_joint(v_key)
        return vj_dict

    # --------------
    # number stats for element(s), virtual joint(s)
    # --------------

    def get_size_of_virtual_joints(self):
        return self._num_of_virtual_joints

    def get_size_of_elements(self):
        return self._num_of_elements

    def get_size_of_grounded_elements(self):
        return len([e for e in self.elements.values() if e.is_grounded])

    # --------------
    # Neighbor / connectivity query
    # --------------

    def e_neighbor_vj(self, e_id, index_only=False):
        nghb_keys = self._net.vertex_neighbors(element_vert_key(e_id))
        if index_only:
            return [extract_virtual_joint_vert_id(vj_key) for vj_key in nghb_keys if extract_virtual_joint_vert_id(vj_key) is not None]
        else:
            return [self._net.get_vertex_attribute(key, 'virtual_joint') for key in nghb_keys if extract_virtual_joint_vert_id(key) is not None]

    def e_neighbor_e(self, e_id, index_only=False):
        nghb_keys = self._net.vertex_neighborhood(element_vert_key(e_id), ring=2)
        if index_only:
            return [extract_element_vert_id(key) for key in nghb_keys if extract_element_vert_id(key) is not None]
        else:
            return [self._net.get_vertex_attribute(key, 'element') for key in nghb_keys if extract_element_vert_id(key) is not None]

    def vj_neighbor_e(self, vj_id, index_only=False):
        nghb_keys = self._net.vertex_neighbors(virtual_joint_key(vj_id))
        if index_only:
            return [extract_element_vert_id(key) for key in nghb_keys if extract_element_vert_id(key) is not None]
        else:
            return [self._net.get_vertex_attribute(key, 'element') for key in nghb_keys if extract_element_vert_id(key) is not None]

    def vj_neighbor_vj(self, vj_id, index_only=False):
        nghb_keys = self._net.vertex_neighborhood(virtual_joint_key(vj_id), ring=2)
        if index_only:
            return [extract_virtual_joint_vert_id(key) for key in nghb_keys if extract_virtual_joint_vert_id(key) is not None]
        else:
            return [self._net.get_vertex_attribute(key, 'virtual_joint') for key in nghb_keys if extract_virtual_joint_vert_id(key) is not None]

    # def elements_shared_virtual_joints(self, e_ids, index_only=False):
    #     shared_keys = set()
    #     for e_id in e_ids:
    #         shared_keys.update()
    #     shared_keys.intersection_update(e2_nghb_keys)
    #     if index_only:
    #         return list(shared_keys)
    #     else:
    #         return [self._net.get_vertex_attribute(key, 'virtual_joint') \
    #             for key in list(shared_keys) if extract_virtual_joint_vert_id(key) is not None]

    # TODO: get_virtual_joint_shared_elements

    # --------------
    # element / virtual joint property query
    # --------------

    def is_element_grounded(self, e_id):
        return self.get_element(e_id).is_grounded

    def get_element_to_ground_dist(self, e_id):
        return self.get_element(e_id).to_ground_dist

    # --------------
    # geometries info query
    # --------------

    @property
    def element_geometries(self):
        return {e_key : element.unit_geometries for e_key, element in self.elements.items()}

    # --------------
    # layer info query
    # --------------

    def get_layer_element_keys(self, layer_id):
        """return list of element int indices of the specified layer
        Parameters
        ----------
        layer_id : int
        Returns
        -------
        list of int
        """
        assert layer_id in self._layer_element_ids.keys()
        return [extract_element_vert_id(e_key) for e_key in self._layer_element_ids[layer_id]]

    def get_layers(self):
        return self._layer_element_ids.keys()

    # --------------
    # static obstacle getter / setter
    # --------------
    # TODO: static obstacles should be removed from this class, should be specified in an URDF

    @property
    def static_obstacle_geometries(self):
        return self._static_obstacle_geometries

    @static_obstacle_geometries.setter
    def static_obstacle_geometries(self, unit_geometries):
        self._static_obstacle_geometries = {ug.name : ug for ug in unit_geometries}

    # --------------
    # exporters
    # --------------

    def save_assembly_to_json(self, json_path, pkg_name='', assembly_type='', model_type='', unit='',
        mesh_path='', json_indent=None):
        if not os.path.isdir(json_path):
            os.mkdir(json_path)
        json_file_path = os.path.join(json_path, pkg_name + '.json')

        data = OrderedDict()
        data['pkg_name'] = pkg_name
        data['assembly_type'] = assembly_type
        data['model_type'] = model_type
        data['unit'] = unit

        data['element_number'] = self.get_size_of_elements()
        data['virtual_joint_number'] = self.get_size_of_virtual_joints()
        data['grounded_number'] = self.get_size_of_grounded_elements()

        data['elements'] = []
        for e_id, e in self.elements.items():
            e_data = OrderedDict()
            e_data['element'] = e.to_data(mesh_path)
            data['elements'].append(e_data)

        # virtual joint data
        data['virtual_joints'] = []
        for vj_id, vj in self.virtual_joints.items():
            vj_data = OrderedDict()
            vj_data['virtual_joint'] = vj.to_data(mesh_path)
            data['virtual_joints'].append(vj_data)

        # TODO: remove later
        static_path = os.path.join(mesh_path, '..', '..', 'static_obstacles', 'collision')
        if not os.path.isdir(static_path):
            os.mkdir(static_path)
        data['static_obstacle_geometries'] = [so_ug.to_data(static_path) for so_ug in self.static_obstacle_geometries.values()]

        with open(json_file_path, 'w') as outfile:
            json.dump(data, outfile, indent=json_indent)

    def save_assembly_to_urdf(self, urdf_path):
        raise NotImplementedError

    def to_package(self, save_path, pkg_name, \
        assembly_type="", model_type="", unit="", json_indent=None):
        root_path = os.path.join(save_path, pkg_name)
        if not os.path.isdir(root_path):
            os.mkdir(root_path)

        json_path = os.path.join(root_path, "json")
        mesh_path = os.path.join(root_path, "meshes", 'assembly_elements', "collision")
        urdf_path = os.path.join(root_path, "urdf")
        check_paths = [json_path, mesh_path, urdf_path]
        for p in check_paths:
            if not os.path.isdir(p):
                os.mkdir(p)

        # generate json
        self.save_assembly_to_json(json_path, pkg_name, assembly_type, model_type, unit, mesh_path, json_indent)

        # TODO: genereate static collision objects urdf
        # self.generate_env_collision_objects_urdf(collision_objs, urdf_path)

    @classmethod
    def from_package(cls, data):
        assembly = cls()
        for e_data in data['elements']:
            e = Element.from_data(e_data['element'])
            assembly.add_element(e)

        # virtual joints
        for vj_data in data['virtual_joints']:
            vj = VirtualJoint.from_data(vj_data['virtual_joint'])
            assembly.add_virtual_joint(vj)

        # static obstacles
        assembly.static_obstacle_geometries = [UnitGeometry.from_data(ug_data) for ug_data in data['static_obstacle_geometries']]
        return assembly

    def __repr__(self):
        return 'assembly_net: #virual joint:{0}, #element:{1}'.format(self.get_size_of_virtual_joints(), self.get_size_of_elements())

    def print_neighbors(self, index_only=True):
        """debug function
        """
        print('#'*10)
        # for e in self.elements.values():
        #     print('*'*6)
        #     c_e_ids = set(self.get_element_neighbored_elements(e.key_id, index_only=index_only))
        #     c_vj_ids = self.get_element_neighbored_virtual_joints(e.key_id, index_only=index_only)
        #     print('grounded:{}'.format(e.is_grounded))
        #     print('e#{0} neighbor e: {1}'.format(e.key_id, c_e_ids))
        #     print('e#{0} neighbor vj: {1}'.format(e.key_id, c_vj_ids))
        #     vj_e_ids = set()
        #     for vj_id in c_vj_ids:
        #         vj_e_ids.update(self.get_virtual_joint_neighbored_elements(vj_id, index_only=True))
        #     print('vj ring neighbor e: {}'.format(vj_e_ids))
        #     assert c_e_ids == vj_e_ids

        # print('#'*10)
        # for vj in self.virtual_joints.values():

        #     print('*'*6)
        #     print('neighbor e of vj#{0}: {1}'.format(vj.key_id, self.get_virtual_joint_neighbored_elements(vj.key_id, index_only=index_only)))

if __name__ == "__main__":
    # Rhino example
    pass
