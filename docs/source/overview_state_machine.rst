Overview of the statemachine
============================

This page gives an overview of the statemachine and its components. It is intended to give a high-level understanding of the statemachine.
For a more detailed description of the statemachine, see :doc:`details_state_machine`. 

The statemachine is a finite state machine that is used to control the behavior of the robot. It is implemented in python with the `smach <http://wiki.ros.org/smach>`_ library.

It currently consists of 5 main components:

* FindGrasp: This component is responsible for calling the pose estimator and finding grasp points for the robot.
* ExecuteGrasp: This component is responsible for executing the grasp(s) that was found by the FindGrasp component. It also updates the collision environment.
* RobotSetup: This component is responsible for moving the robot and all of its joints into a predefined position.
* Placement: This component is responsible for placing the grasped, known object to its appointed location.
* Handover: This component is responsible for handing over the grasped object to a human.

Each of these components is implemented as a state machine (and therefore consists of multiple states), and the components are all combined into a single, big state machine that controls the behavior of the robot.

=========
FindGrasp
=========
The FindGrasp component calls the pose estimator to get the estimated poses for each of the objects detected in the scene. 

In the case of known object pose estimation, it uses the name to lookup the grasp annotations for that object. In the case of unknown object pose estimation an unknown object grasp pose estimator is called to get the grasp pose. 


Outcomes
--------
* **succeeded**: The grasp was found successfully.
* **aborted**: No object or grasp was found.
* **preempted**: This outcome is currently not used.

Inputs
------
* **object_to_grasp (string)**: The name of the object to grasp. If no name is given (=empty string), the closest object is grasped instead.

Outputs
-------
* **grasp_poses (list of geometry_msgs/PoseStamped)**: A list of grasp poses for the object that should be grasped.
* **grasp_object_bb (grasping_pipeline_msgs/BoundingBox3DStamped)**: The bounding box of the grasped object. This is used for collision checking when placing the object.
* **grasp_object_name (string)**: The name of the grasped object. This name is needed for placement to know where to place the object, as each object has a predefined placement area.

============
ExecuteGrasp
============
The ExecuteGrasp component is responsible for executing the grasp that was found by the FindGrasp component.

First, the table plane that the object is resting on is detected. Then, the collision environment is updated with the `grasp_object_bb` and the detected table. The `grasp_object_bb` is needed if placement should be done after the grasping. The table is needed to prevent the robot from colliding with the table. 
  
Afterwards, the robot moves to the grasp pose and executes the grasp. Simultaneously, it records the transformation between the robot's end-effector and the object's bottom plane. This transformation is needed for placement to ensure that the object is placed in a manner that maintains the objects original orientation (i.e. the bottom side of the object when it was grasped, will also be the bottom side of the object after it is placed).

If the grasp failed, the component returns a 'failed_to_grasp' outcome.

If the grasp was succesful, the robot retreats from the table, moves the joints into a neutral position and drives to a predefined position in a way that should prevent the robot and its arm from coliding with any object (especially the table). Afterwards, 'execute_grasp_success' is returned.

Outcomes
--------
* **failed_to_grasp**: Execution of all provided grasp poses failed.
* **execute_grasp_success**: The grasp was executed successfully.

Inputs
------
* **grasp_poses (list of PoseStamped)**: A list of grasp poses for the object that should be grasped. The grasp_poses are processed iteratively until the grasp was succesful or no further grasp pose remains.
* **grasp_object_bb (BoundingBox3DStamped)**: The bounding box of the grasped object. This is used for collision checking when placing the object.

Outputs
-------
* **placement_surface_to_wrist (geometry_msgs/Transform)**: The transformation between the robot's end-effector and the object's bottom plane. This transformation is needed for placement to ensure that the object is placed in a manner that maintains the objects original orientation.

==========
RobotSetup
==========

This component is responsible for moving the robot and all of its joints into a predefined position. This is needed to ensure that the robot is in a known state before any other state is executed. 

More specifically, the robot moves in front of the table, the arm is moved in a way so that it does not cover parts of the camera, the gripper is opened and the head is moved so that it gazes at the table plane.

Outcomes
--------
* **setup_succeeded**: The robot and its joints were moved into the predefined position successfully.

Inputs
------
None

Outputs
-------
None


=========
Placement
=========

========
Handover
========

The Handover component is responsible for handing over the grasped object to a human. The robot's arm is stretched out to make it as comfortable as possible for adults to take the object. The torque sensor in the robot's wrist is used to detect when the object is taken by the human. When the torque sensor detects a force above a certain threshold, the robot releases the object and returns 'succeeded'.

Outcomes
--------
* **succeeded**: The object was handed over successfully.
* **aborted**: This outcome is currently not used.
* **preempted**: This outcome is currently not used.

Inputs
------
* **force_thresh (float), optional**: The threshold for the torque sensor to detect when the object is taken by the human. A default value is used if no threshold is passed or the passed threshold is <0.

Outputs
-------
None

============
Other states
============

Some states are currently not used in the statemachine, but are still implemented. These states are:

*
*
