from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

import os
import math
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

    """

    __module__ = 'compas_assembly.datastructures'

    def __init__(self, elements=None, attributes=None):
        super(Assembly, self).__init__()
        self._net = Network()
        self._element_geometries = {} # object frame moved to world frame
        self._virtual_joint_geometries = {}
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

    def add_element(self, element_instance, unit_geometry):
        """[summary]

        Parameters
        ----------
        element_instance : compas.assembly.datastructures.Element
            [description]
        unit_geometry : compas.assembly.datastructures.UnitGeometry
            [description]
        """
        # unit geometry's object frame transformed to origin
        self._net.add_vertex(key=element_instance.key, element=element_instance, tag='element')
        self._element_geometries[element_instance.key] = unit_geometry

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
            e_dict[extract_element_vert_id(v_key)] = self.get_element(v_key)
        return e_dict

    def add_virtual_joint(self, vj_instance, unit_geometry=None):
        """
        Network edges are added here.

        Parameters
        ----------
        vj_instance : compas.assembly.datastructures.VirtualJoint
            [description]
        unit_geometry : [type], optional
            [description], by default None
        """
        self._net.add_vertex(key=vj_instance.key, virtual_joint=vj_instance, tag='virtual_joint')
        connected_e_ids = vj_instance.connected_element_ids
        for e_id in connected_e_ids:
            # assert(element_vert_key(e_id) in self._net.vertex)
            self._net.add_edge(element_vert_key(e_id), vj_instance.key)
        if unit_geometry:
            self._virtual_joint_geometries[virtual_joint_key(id)] = unit_geometry
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
            vj_dict[extract_virtual_joint_vert_id(v_key)] = self.get_element(v_key)
        return vj_dict

    # --------------
    # number stats for element(s), virtual joint(s)
    # --------------

    def get_size_of_virtual_joints(self):
        return self._num_of_virtual_joints

    def get_size_of_elements(self):
        return self._num_of_elements

    def get_size_of_grounded_elements(self):
        return len([e for e in self.elements if e.is_grounded])

    # --------------
    # Neighbor / connectivity query
    # --------------

    def get_element_neighbored_virtual_joints(self, e_id):
        nghb_keys = self._net.vertex_neighbors(element_vert_key(e_id))
        return [self._net.get_vertex_attribute(vj_key, 'virtual_joint') for vj_key in nghb_keys]

    def get_element_neighbored_elements(self, e_id):
        nghb_keys = self._net.vertex_neighborhood(element_vert_key(e_id), ring=1)
        return [self._net.get_vertex_attribute(key, 'elements') for key in nghb_keys if extract_element_vert_id(key) is not None]

    def get_element_shared_virtual_joints(self, e1_id, e2_id):
        """get virtual joint shared by the two input elements

        Parameters
        ----------
        e1_id : int
            [description]
        e2_id : int
            [description]

        Returns
        -------
        VirtualJoint
            [description]
        """
        e1_nghb_keys = self._net.vertex_neighbors(element_vert_key(e1_id))
        e2_nghb_keys = self._net.vertex_neighbors(element_vert_key(e2_id))
        shared_keys = set(e1_nghb_keys)
        shared_keys.intersection_update(e2_nghb_keys)
        return [self._net.get_vertex_attribute(key, 'virtual_joint') for key in list(shared_keys) if extract_virtual_joint_vert_id(key) is not None]

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

    def get_element_geometry_in_pick_pose(self, e_id):
        """return shape geometries in pick pose"""
        e = self.get_element(e_id)
        assert(e.world_from_element_pick_pose != None, "pick pose not defined!")
        world_pick_tf = Transformation.from_frame(e.world_from_element_pick_pose)
        return [transform_cmesh(cm, world_pick_tf) \
        for cm in self._element_geometries[element_vert_key(e_id)]]

    def get_element_geometry_in_place_pose(self, e_id):
        """return shape geometries in place pose"""
        e = self.get_element(e_id)
        assert(e.world_from_element_place_pose != None, "place pose not defined!")
        world_place_tf = Transformation.from_frame(e.world_from_element_place_pose)
        return [transform_cmesh(cm, world_place_tf) \
        for cm in self._element_geometries[element_vert_key(e_id)]]

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
    # some built-in graph algorithm
    # --------------

    def dijkstra(self, src_e_id, sub_graph=None):
        def min_distance(e_size, dist, visited_set):
            # return -1 if all the unvisited vertices' dist = inf (disconnected)
            min = math.inf
            min_index = -1
            for e_id in range(e_size):
                if dist[e_id] < min and not visited_set[e_id]:
                    min = dist[e_id]
                    min_index = e_id
            # assert(min_index > -1)
            return min_index

        if self.is_element_grounded(src_e_id):
            return 0
        e_size = self.get_size_of_elements()
        dist = [math.inf] * e_size
        dist[src_e_id] = 0
        visited_set = [False] * e_size

        for k in range(e_size):
            if sub_graph:
                if k not in sub_graph:
                    continue
            e_id = min_distance(e_size, dist, visited_set)
            if e_id == -1:
                # all unvisited ones are inf distance (not connected)
                break

            visited_set[e_id] = True

            nbhd_e_ids = set(self.get_element_neighbor(e_id))
            if sub_graph:
                nbhd_e_ids = nbhd_e_ids.intersection(sub_graph)

            for n_e_id in nbhd_e_ids:
                if not visited_set[n_e_id] and dist[n_e_id] > dist[e_id] + 1:
                    dist[n_e_id] = dist[e_id] + 1

        # get smallest dist to the grounded elements
        grounded_dist = [dist[e.e_id] for e in self.assembly_elements.values() if e.is_grounded]
        return min(grounded_dist)

    # @timecall
    def compute_traversal_to_ground_dist(self, sub_graph=None):
        considered_e_ids = self.assembly_elements.keys() if not sub_graph else sub_graph
        for e in considered_e_ids:
            self.assembly_elements[e].to_ground_dist = self.dijkstra(e, sub_graph)

    # --------------
    # static obstacle getter / setter
    # --------------

    @property
    def static_obstacle_geometries(self):
        return self._static_obstacle_geometries

    @static_obstacle_geometries.setter
    def static_obstacle_geometries(self, cmesh_lists):
        # assert(isinstance(cmesh_lists, list))
        self._static_obstacle_geometries = {STATIC_OBSTACLE_PREFIX + '_' + str(id) : cl for id, cl in enumerate(cmesh_lists)}

    # --------------
    # exporters
    # --------------

    def save_element_geometries_to_objs(self, mesh_path):
        if not os.path.isdir(mesh_path):
            os.mkdir(mesh_path)
        for eg_key, eg_val in self._element_geometries.items():
            for sub_id, cm in enumerate(eg_val):
                cm.to_obj(os.path.join(mesh_path, obj_name(eg_key, sub_id)))

    def save_static_obstacles_geometries_to_objs(self, mesh_path):
        if not os.path.isdir(mesh_path):
            os.mkdir(mesh_path)
        for eg_key, eg_val in self.static_obstacle_geometries.items():
            for sub_id, cm in enumerate(eg_val):
                cm.to_obj(os.path.join(mesh_path, obj_name(eg_key, sub_id)))

    def save_assembly_to_json(self, json_path, pkg_name='', assembly_type='', model_type='', unit='', given_seq=None):
        if not os.path.isdir(json_path):
            os.mkdir(json_path)
        json_file_path = os.path.join(json_path, pkg_name + '.json')

        data = OrderedDict()
        data['pkg_name'] = pkg_name
        data['assembly_type'] = assembly_type
        data['model_type'] = model_type
        data['unit'] = unit
        data['with_given_sequence'] = bool(given_seq)
        # TODO: sanity check: vj geo + element geo = given seq

        data['element_number'] = self._num_of_elements
        data['sequenced_elements'] = OrderedDict()

        for e, order_id in zip(self.elements.values(), given_seq):
            e_data = OrderedDict()
            e_data['order_id'] = order_id
            e_data['object_id'] = e.key
            e_data['parent_frame'] = Transformation.from_frame(e.parent_frame).list
            e_data['element_geometry_file_names'] = {sub_id : {'full_obj' : obj_name(e.key, sub_id)} \
                for sub_id in range(len(self._element_geometries[e.key]))}

            e_data['assembly_process'] = OrderedDict()
            pick = OrderedDict()
            pick['process_name'] = 'pick'
            pick['parent_frame'] = Transformation.from_frame(e.parent_frame).list
            pick['object_target_pose'] = Transformation.from_frame(e.world_from_element_pick_pose).list
            pick['allowed_collision_obj_names'] = [] # support tables, neighbor elements?
            # old names: pick_contact_ngh_ids, pick_support_surface_file_names
            pick['grasp_from_approach_tf'] = e.grasp_from_approach_tfs[0].list
            e_data['assembly_process']['pick'] = pick

            place = OrderedDict()
            place['process_name'] = 'place'
            place['parent_frame'] = Transformation.from_frame(e.parent_frame).list
            place['object_target_pose'] = Transformation.from_frame(e.world_from_element_place_pose).list
            place['allowed_collision_obj_names'] = [] # support tables
            place['grasp_from_approach_tf'] = e.grasp_from_approach_tfs[0].list
            e_data['assembly_process']['place'] = place
            # TODO: neighbor ACM
            # for nghb_id in self.vertex_neighborhood(v):
            #     place['allowed_collision_obj_names'].extend(
            #         [obj_name(nghb_id, sub_id) \
            #             for sub_id in range(len(self.assembly_object(nghb_id).shape))])

            grasp_data = OrderedDict()
            grasp_data['parent_link'] = 'object'
            grasp_data['ee_poses'] = [Transformation.from_frame(ee_p).list \
                for ee_p in e.obj_from_grasp_poses]

            e_data['grasps'] = grasp_data
            data['sequenced_elements'][e.key] = e_data

        data['static_obstacles'] = OrderedDict()
        for cl_key, cl in self.static_obstacle_geometries.items():
            data['static_obstacles'][cl_key] = OrderedDict()
            for sub_id in range(len(cl)):
                data['static_obstacles'][cl_key][sub_id] = OrderedDict()
                data['static_obstacles'][cl_key][sub_id]['full_obj'] = \
                obj_name(cl_key , sub_id)

        with open(json_file_path, 'w') as outfile:
            json.dump(data, outfile, indent=4)

    # def save_assembly_to_urdf(self, urdf_path):
    #     pass

    def save_to_assembly_planning_pkg(self, save_path, pkg_name, \
        assembly_type="", model_type="", unit="", given_seq=None):
        root_path = os.path.join(save_path, pkg_name)
        if not os.path.isdir(root_path):
            os.mkdir(root_path)

        json_path = os.path.join(root_path, "json")
        mesh_path = os.path.join(root_path, "meshes", "collision")
        urdf_path = os.path.join(root_path, "urdf")
        check_paths = [json_path, mesh_path, urdf_path]
        for p in check_paths:
            if not os.path.isdir(p):
                os.mkdir(p)

        # generate obj files
        self.save_element_geometries_to_objs(mesh_path)
        self.save_static_obstacles_geometries_to_objs(mesh_path)

        # generate json
        self.save_assembly_to_json(json_path, pkg_name, assembly_type, model_type, unit, given_seq)

        # TODO: genereate static collision objects urdf
        # self.generate_env_collision_objects_urdf(collision_objs, urdf_path)

    @classmethod
    def from_assembly_planning_pkg(cls):
        pass

    def __repr__(self):
        return 'assembly_net: #virual joint:{0}, #element:{1}'.format(self.get_size_of_virtual_joints(), self.get_size_of_elements())

    def print_neighbors(self):
        """debug function
        """
        for e in self.assembly_elements.values():
            print('grounded:{}'.format(e.is_grounded))
            print('neighbor e of e{0}:{1}'.format(e.e_id, self.get_element_neighbor(e.e_id)))
            for end_id in e.node_ids:
                print('neighbor e of end{0}/e{1}:{2}'.format(
                    end_id, e.e_id, self.get_element_neighbor(e.e_id, end_id)))

        for v in self.assembly_joints.values():
            print('neighbor e of v{0}: {1}'.format(v.node_id, self.get_node_neighbor(v.node_id)))

if __name__ == "__main__":
    pass
    # from compas.datastructures import Mesh
    # from compas_fab.assembly import Element

    # assembly = Assembly()
    # mesh = Mesh.from_polyhedron(4)

    # for i in range(2):
    #     element = Element.from_mesh(mesh)
    #     assembly.add_element(element)

    # print(assembly.summary())
