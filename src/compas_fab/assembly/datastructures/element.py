from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

import json
import math

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
        self.to_ground_dist = math.inf

    @property
    def key(self):
        return self._key

    @property
    def key_id(self):
        return extract_element_vert_id(self.key)

    @property
    def layer_id(self):
        return self._layer_id

# ==============================================================================
# Main
# ==============================================================================
if __name__ == "__main__":
    # e = Element(id)
    # e.world_from_element_place_pose = rPln2cFrame(world_from_place_poses[id])
    # e.world_from_element_pick_pose = rPln2cFrame(world_from_pick_poses[id])
    # print('{}, {}'.format(id, len(grasp_pose_lists[id])))
    # cgrasp_poses = [rPln2cFrame(gpose) for gpose in grasp_pose_lists[id]]
    # e.set_grasp_poses_from_in_scene_poses(e.world_from_element_pick_pose, cgrasp_poses)
    # capproach_poses = [rPln2cFrame(apppose) for apppose in approach_pose_lists[id]]
    # e.set_approach_poses_world(cgrasp_poses, capproach_poses)
    # cmesh_list = [transform_cmesh_to_origin(rMesh2cMesh(rm), e.world_from_element_place_pose) for rm in rm_list]
    # asm.add_element(e, id, cmesh_list)
