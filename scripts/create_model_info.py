#! /usr/bin/env python3
import os
import yaml
import open3d as o3d
import numpy as np

dataset_name = "ycb_bop"
models_directory = '../models/'
model_files_directory = os.path.join(models_directory, dataset_name)
result_file_path = '../models/models_metadata.yml'
show_visualization = True
model_to_m_conversion_factor = 1/1000.0 # e.g. 1/1000.0 to convert to m if model is in mm
wanted_files = None # None to process all files, or a list of filenames to process only those

if __name__ == '__main__':
    print("You will be asked to choose between the axis aligned bounding box (red) and the oriented bounding box (blue) for each model.")
    print("Please choose the one that fits the model better.")
    metadata = {}
    if os.path.exists(result_file_path):
        with open(result_file_path, 'r') as f:
            metadata = yaml.load(f, Loader=yaml.FullLoader)
    metadata[dataset_name] = {}
    for filename in os.listdir(model_files_directory):
        if wanted_files is not None and filename not in wanted_files:
            continue
        filepath = os.path.join(model_files_directory, filename)
        model_name = filename.replace('.stl', '').replace('.ply', '')
        if model_name in metadata:
            print(f"Skipping file {filepath} because it is already in the metadata.")
            continue
        if filename.endswith('stl') or filename.endswith('ply'):
            print(f"Working on file {filepath}")
            model = o3d.io.read_triangle_mesh(filepath)
            # o3d bug: get_minimal_oriented_bounding_box does not work when directly called on the mesh -> convert to point cloud first
            # should be fixed in 0.19 (fix already merged in master)
            or_bb = model.sample_points_uniformly(5000, use_triangle_normal=True).get_minimal_oriented_bounding_box(robust = True)
            aa_bb = model.get_axis_aligned_bounding_box()
            if show_visualization:
                or_bb.color = [0.0, 0.0, 1.0]
                aa_bb.color = [1.0, 0.0, 0.0]
                o3d.visualization.draw_geometries([model, or_bb, aa_bb])
            while True:
                # Red: axis aligned bounding box; Blue: oriented bounding box
                print("Red or Blue better fit? (r/b)")
                c = input()
                if c == 'r':
                    rot_mat = np.eye(3).tolist()
                    center = (aa_bb.get_center() * model_to_m_conversion_factor).tolist()
                    extent = (aa_bb.get_extent() * model_to_m_conversion_factor).tolist()
                    break
                elif c == 'b':
                    rot_mat = or_bb.R.tolist()
                    center = (or_bb.get_center() * model_to_m_conversion_factor).tolist()
                    extent = (or_bb.extent * model_to_m_conversion_factor).tolist()
                    break
            obj_data = {'center': center, 'rot': rot_mat, 'extent': extent}
            metadata[dataset_name][model_name] = obj_data
        else:
            print(f"Skipping file {filepath}")
    with open(result_file_path, "w") as f:
        yaml.dump(metadata, f)