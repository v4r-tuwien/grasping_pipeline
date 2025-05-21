Adding new objects to the grasping pipeline
===========================================

* Copy the object meshes
    Copy the meshes to the `grasping_pipeline/models/{DATASET_NAME}` directory. The meshes should either be in `.stl` or `.ply` format and must be scaled to `mm` (i.e. 1 'unit' of the mesh should equal 1 mm).
    The name of each mesh should be the same as the names that the object detector and pose estimator return. 
* Copy the grasp annotations
    Copy the grasp annotations to the `grasping_pipeline/grasps/{DATASET_NAME}` directory. The grasp annotations should be saved as a '.npy' file and should contain a numpy array of shape `(N, 1, 16)`, where `N` is the number of grasps and each row contains the flattened 4x4 transformation matrix. These annotations can be created with `this blender script <https://github.com/v4r-tuwien/grasp_annotation_blender>`_.
* Run the `create_model_info.py` script
    Update the `create_model_info.py` script with the new dataset name and run it. The script will show you two different bounding boxes and you have to choose which one fits better.
* Update the `config/placement_areas.yaml` config file (optional)
    Add the new objects to the config file and define the placement area for each objects. This is only necessary if you want the robot to be able to place the objects.
* Update the `grasping_pipeline/config/config.yaml` file
    Update the `dataset` entry in the `grasping_pipeline/config/config.yaml` file with the new dataset name. Alternatively, you can update the `grasping_pipeline/dataset` rosparam-parameter.