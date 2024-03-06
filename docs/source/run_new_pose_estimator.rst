Running a new pose estimator with the grasping pipeline
=======================================================

To make a new pose estimation model compatible with the grasping pipeline you will need to create a ROS Wrapper for the model (we might provide a template for this in the future).

The ROS Wrapper should expose an action server that uses the `robokudo_msgs/GenericImgProcAnnotatorAction`: 

.. code-block:: yaml

    GenericImgProcAnnotatorAction:
    #goal
    sensor_msgs/Image rgb
    sensor_msgs/Image depth
    sensor_msgs/RegionOfInterest[] bb_detections
    sensor_msgs/Image[] mask_detections
    string[] class_names
    string description
    ---
    #result
    bool success
    string result_feedback
    sensor_msgs/RegionOfInterest[] bounding_boxes
    int32[] class_ids
    string[] class_names
    float32[] class_confidences
    sensor_msgs/Image image
    geometry_msgs/Pose[] pose_results
    string[] descriptions
    ---
    #feedback
    string feedback

Currently, the grasping pipeline only passes **rgb** and **depth** to the pose estimation model. 

To get the camera intrinsics, your wrapper should listen to the topic specified by the `cam_info_topic` rosparam parameter.

The result you should return depends on whether the model is a known or unknown object pose estimator.

============================
Known object pose estimation
============================

In the case of known object pose estimation, the result should contain the following fields:

* **class_names**: The name of the object that was detected. This has to be consistent with the names used in the `grasping_pipeline/models` and `grasping_pipeline/grasps` directories, as the names are used for looking up the grasp annotations and model bounding_boxes.
* **pose_results**: This should contain the pose of each object relative to the rgb/depth camera frame. 
* **class_confidences (optional)**: The confidence of the pose estimation or detection. This is used to filter out low confidence estimations. If no confidences are passed, no filtering happens. 

.. note::
   The grasping pipeline uses the `class_names` to look up the grasp annotations. The `pose_results` are then transformed into `grasping_poses` by using the grasp annotations.

==============================
Unknown object pose estimation
==============================

In the case of unknown object pose estimation, the result should contain the following fields:

* **class_names**: The name of each unknown object should be set to "Unknown" (Uppercase U!).
* **image**: This should be an image with the same dimensions as the input rgb image. The image should be a label image (with dtype=int) where each object pixel is labeled with a unique id (eg. pixels of object A == 1, pixels of object B == 2). "No Object" pixels MUST be encoded with -1. 

.. note:: 
   The grasping pipeline calculates the object bounding boxes from the label image. The bounding boxes are then passed to an unknown object grasp pose estimator like `HAF grasping <https://github.com/v4r-tuwien/haf_grasping>`_ to get the grasp poses.


=====================================
Updating the grasping pipeline config
=====================================

To actually use the new pose estimator, you will need to update the `grasping_pipeline/config/config.yaml` file:

.. code-block:: yaml

    pose_estimator:
      detector_topic: /TOPIC_OF_YOUR_POSE_ESTIMATOR
      class_confidence_threshold: 0.3

The `detector_topic` should be set to the topic of the action server of your pose estimator. The `class_confidence_threshold` is used to filter out low confidence detections. If your pose estimator does not return confidences, you can ignore this value.
