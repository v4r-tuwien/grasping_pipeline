# Grasping Pipeline
Grasping pipeline for grasping, placing and handing over objects with the HSR robot. The pipeline is implemented in ROS.

Link to the [Documentation](https://v4r-tuwien.github.io/grasping_pipeline/)

- [Installation](https://v4r-tuwien.github.io/grasping_pipeline/installation.html)
- [Starting the grasping pipeline](https://v4r-tuwien.github.io/grasping_pipeline/startup.html)
- [Integrating a new Detector or Pose Estimator](https://v4r-tuwien.github.io/grasping_pipeline/run_new_pose_estimator.html)
- [Overview of the statemachine](https://v4r-tuwien.github.io/grasping_pipeline/overview_state_machine.html)
- [API](https://v4r-tuwien.github.io/grasping_pipeline/api.html)

## Grasp Annotations
The grasp annotations are stored as numpy files in the /grasps folder. The array consists of float64 values in the shape (n, 1, 16), where n is the number of grasps stored. One grasp is represented by the 4x4 transformation matrix. The naming of the file name has to match the naming of the object. 

You can create new grasp annotations with the help of [this repo](https://github.com/v4r-tuwien/grasp_annotation_blender).
