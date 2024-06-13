Adding a new estimator or detector to the grasping pipeline
===========================================================

The grasping pipeline is designed to be modular and can be extended with new object detection, pose estimation or grasp pose estimation models. 
This document describes how to add new models to the grasping pipeline.

The model should be ROS compatible and should expose an action server that uses the `robokudo_msgs/GenericImgProcAnnotatorAction` action message.

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

Depending on the type of model you want to add, you will get different goals and are expected to return different results.

============================
Adding a new object detector
============================

To make a new object detection model compatible with the grasping pipeline you will need to create a ROS Wrapper for the model.

The ROS Wrapper should expose an action server that uses the `robokudo_msgs/GenericImgProcAnnotatorAction` mentioned above.

The object detector will only get the **rgb** and **depth** images as input.

The object detector is expected to return the following fields:

* **bounding_boxes(optional)**: The bounding boxes of the detected objects in the image. 
* **class_names**: The name of the detected objects. In the case of known objects, the name is used to look up the grasp annotations.
   Therefore, the names have to be identical to the names used in the `grasping_pipeline/models` and `grasping_pipeline/grasps` directories.
   In the case of unknown objects, the name should be set to "Unknown" (Uppercase U!).
* **class_confidences (optional)**: The confidence of the detection.
* **image (optional)**: This should be an image with the same dimensions as the input rgb image. 
   The image should be a label image (with dtype=int) where each object pixel is labeled with a unique id (eg. pixels of object A == 1, pixels of object B == 2). "No Object" pixels MUST be encoded with -1.
   So basically, the image shows the segmentation masks of the detected objects.

The object detector should return either the bounding boxes or the image. If only the image is returned, the grasping pipeline will calculate the bounding boxes from the label image.

To get the camera intrinsics, your wrapper should listen to the topic specified by the `cam_info_topic` rosparam parameter.

===========================
Adding a new pose estimator
===========================

To make a new pose estimation model compatible with the grasping pipeline you will need to create a ROS Wrapper for the model.

The ROS Wrapper should expose an action server that uses the `robokudo_msgs/GenericImgProcAnnotatorAction` mentioned above.

The pose estimator will get the following fields as input:

* **rgb**: The rgb image of the scene.
* **depth**: The depth image of the scene.
* **bb_detections**: The bounding boxes of the detected objects in the image.
* **mask_detections (optional)**: The segmentation masks of the detected objects in the image.
* **class_names**: The name of the detected objects. In the case of unknown objects, the name is set to "Unknown" (Uppercase U!).
* **description**: A string in json format, which is used to pass the confidence scores from the object detector. The format is as follows: "{obj_name: confidence, obj_name2: confidence}". 
   You can simply use the json library to parse the string and get a python dictionary that you can index with the object names to get the confidence scores.

The pose estimator is expected to return the following fields:

* **pose_results**: The pose of each object relative to the rgb/depth camera frame.
* **class_names**: The name of the objects whose pose was estimated. The name is used to look up the grasp annotations. Therefore, the names have to be identical to the names used in the `grasping_pipeline/models` and `grasping_pipeline/grasps` directories.
* **class_confidences (optional)**: The confidence of the objects whose pose was estimated. If your pose estimator does not return confidences, you can set the confidence to 1.0 for each object or you can pass the confidences from the object detector (but make sure that you only return the confidences from the object who actually got a pose estimation).

To get the camera intrinsics, your wrapper should listen to the topic specified by the `cam_info_topic` rosparam parameter.

=================================
Adding a new grasp pose estimator
=================================

To make a new grasp pose estimation model compatible with the grasping pipeline you will need to create a ROS Wrapper for the model.

The ROS Wrapper should expose an action server that uses the `robokudo_msgs/GenericImgProcAnnotatorAction` mentioned above.

The grasp pose estimator will get the following fields as input:

* **rgb**: The rgb image of the scene.
* **depth**: The depth image of the scene.
* **bb_detections**: The bounding boxes of the detected objects in the image.
* **mask_detections (optional)**: The segmentation masks of the detected objects in the image.
* **class_names**: The name of the detected objects. In the case of unknown objects, the name is set to "Unknown" (Uppercase U!).

The grasp pose estimator is expected to return the following fields:

* **pose_results**: The grasp pose of each object relative to the rgb/depth camera frame.
* **class_names**: The name of the objects whose grasp pose was estimated.
* **class_confidences (optional)**: The confidence of the objects whose grasp pose was estimated. If your grasp pose estimator does not return confidences, you can set the confidence to 1.0 for each object or you can pass the confidences from the object detector (but make sure that you only return the confidences from the object who actually got a grasp pose estimation).


To get the camera intrinsics, your wrapper should listen to the topic specified by the `cam_info_topic` rosparam parameter.

=====================================
Updating the grasping pipeline config
=====================================

To actually use the new pose estimator, you will need to update the `grasping_pipeline/config/config.yaml` file and restart the grasping pipeline.

.. code-block:: yaml

   grasping_pipeline:
      object_detector_topic: /object_detector/yolov5                      # object detector for known OR unknown objects
                                                                          # decides whether a pose estimator or a grasp point 
                                                                          # estimator is used afterwards
      pose_estimator_topic: /pose_estimator/gdrnet                        # pose estimator for known objects
      grasppoint_estimator_topic: /pose_estimator/find_grasppose_haf      # grasppoint estimator for unknown objects

The object_detector_topic should be set to the topic of the new object detector.

The pose_estimator_topic should be set to the topic of the new pose estimator.

The grasppoint_estimator_topic should be set to the topic of the new grasp pose estimator.