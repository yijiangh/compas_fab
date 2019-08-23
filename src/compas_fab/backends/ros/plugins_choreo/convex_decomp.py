from compas.datastructures import Mesh

def convex_decomp_from_mesh_files(mesh_file_name, output_path='', log_path='', export_separate_files=False, decomp_res=1e5, verbose=False):
    try:
        from py_vhacd import compute_convex_decomp
    except ImportError as e:
        print('\x1b[6;30;43m' + '{}'.format(e) + '\x1b[0m')
        raise ImportError

    fail, mesh_verts, mesh_faces = compute_convex_decomp(mesh_file_name, output=output_path, log=log_path, \
        resolution=int(decomp_res), export_separate_files=export_separate_files, verbose=verbose)
    if verbose:
        print('decompose success: ', not fail)
        cmeshes = []
        print('total decomp parts: {}'.format(len(mesh_verts)))
        for verts, faces in zip(mesh_verts, mesh_faces):
            print('vert num: {}, face num: {}'.format(len(verts), len(faces)))
            cmeshes.append(Mesh.from_vertices_and_faces(verts, faces))

    if not fail:
        return cmeshes
    else:
        return None
