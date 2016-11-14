#!/usr/bin/env python

""" wrench_correction.py - Version 1.0 2016-02-28


    Created for the MBZIRC competition

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from base_nav import BaseArmNavigation
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped, PointStamped, PoseArray

class WrenchCorrection(BaseArmNavigation):
    def __init__(self, node_name):
        super(DrillArmNavigation, self).__init__(node_name)

    def perform_task(self):
        rospy.loginfo("Performing wrench grasping")

        #Set the target pose to the initial location in the reference frame
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = 1.0
        target_pose.pose.position.y = -0.09
        target_pose.pose.position.z = 0.43
        target_pose.pose.orientation = self.desired_orientation

        outcome = self.arm_nav_predicition(target_pose)
        if outcome:
            rospy.loginfo("Successfully moved to prediction position " + str(location))
            rospy.sleep(2)
            target_pose = PoseStamped()
            target_pose.header.frame_id = self.reference_frame
            target_pose.header.stamp = rospy.Time.now()
            target_pose.pose.position.x = 1.05
            target_pose.pose.position.y = -0.08
            target_pose.pose.position.z = 0.425
            target_pose.pose.orientation = self.desired_orientation
            if target_pose is not None and len(target_pose) > 0:
                target_pose = self.convert_point_to_arm_goal(target_pose, target_pose)
                outcome = self.arm_nav_correction(target_pose)
                if outcome:
                    rospy.loginfo("Successfully updated wrench location ")
                    self.arm.shift_pose_target(0, 0.1, self.end_effector_link)
                    plan = self.arm.plan()
                    if plan is not None:
                        self.arm.execute(plan)
                        rospy.loginfo("Successfully approached the wrench")
                    else:
                        rospy.loginfo("Unable to approach wrench")
                        continue
                else:
                    rospy.loginfo("Unable to update wrench location")
                    continue
            else:
                rospy.loginfo("Unable to locate hole")
                continue


if __name__ == '__main__':
    try:
        node_name = "wrenchl_arm_nav"
        WrenchCorrection(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down the Wrench Correction node."
