#! /usr/bin/env python

import rospy
import actionlib

from grasping_pipeline.msg import FindGrasppointAction, FindGrasppointActionResult
import markus_grasp_test

class FindGrasppointServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('find_grasppoint', FindGrasppointAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
    ## method 1 uses yolo and haf grasping
    if goal.method == 1:
      result = FindGrasppointActionResult().result
      markus = markus_grasp_test.Main()
      rough_grasp_object_center = markus.get_grasp_object_center_yolo()
      markus.call_haf_grasping(rough_grasp_object_center)
      grasp_pose = markus.convert_grasp_result_for_moveit()
      result.grasp_pose = grasp_pose
      self.server.set_succeeded(result)
    else:
      rospy.loginfo('Method not implemented')
      self.server.set_aborted()
if __name__ == '__main__':
  rospy.init_node('find_grasppoint_server')
  server = FindGrasppointServer()
  rospy.spin()