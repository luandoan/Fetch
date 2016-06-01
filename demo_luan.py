# -*- coding: utf-8 -*-
"""
Created on Tue May 31 15:16:14 2016

@author: luan
"""

import rospy
import actionlib
import time

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryDemo():
    def __init__(self):
        rospy.init_node('trajectory_demo')

        # Set to True to move back to the starting configurations
        #reset = rospy.get_param('~reset', False)

        # Which joints define the torso?
        torso_joints = ["torso_lift_joint"]

        # Set to False to wait for arm to finish before moving head
        sync = rospy.get_param('~sync', True)

        # Which joints define the arm?
        arm_joints = ['shoulder_pan_joint',
                      'shoulder_lift_joint',
                      'upperarm_roll_joint',
                      'elbow_flex_joint',
                      'forearm_roll_joint',
                      'wrist_flex_joint',
					  'wrist_roll_joint',
                      ]

        # Which joints define the head?
        head_joints = ['head_pan_joint', 'head_tilt_joint']

        idx = 1
        while (idx < 5):
            
            # Step 1 - discover surrounding environment
            # Set a goal configuration for the head - step 1
            head_goal = [1.0, 0.5]

            # Connect to the head trajectory action server
            rospy.loginfo('Waiting for head trajectory controller...')
            head_client = actionlib.SimpleActionClient('head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            head_client.wait_for_server()
            rospy.loginfo('...connected.')

            # Create a single-point head trajectory with the head_goal as the end-point
            head_trajectory = JointTrajectory()
            head_trajectory.joint_names = head_joints
            head_trajectory.points.append(JointTrajectoryPoint())
            head_trajectory.points[0].positions = head_goal
            head_trajectory.points[0].velocities = [0.0 for i in head_joints]
            head_trajectory.points[0].accelerations = [0.0 for i in head_joints]
            head_trajectory.points[0].time_from_start = rospy.Duration(2.0)

            # Send the trajectory to the head action server
            rospy.loginfo('Moving the head to goal position...')

            head_goal = FollowJointTrajectoryGoal()
            head_goal.trajectory = head_trajectory
            head_goal.goal_time_tolerance = rospy.Duration(0.0)

            # Send the goal
            head_client.send_goal(head_goal)

            # Wait for up to 5 seconds for the motion to complete
            head_client.wait_for_result(rospy.Duration(2.0))
        
            rospy.loginfo('... to the left - head down')
        
            # step 2
            head_goal = [1.0, -0.5]
        
            # Connect to the head trajectory action server
            #rospy.loginfo('Waiting for head trajectory controller...')
            head_client = actionlib.SimpleActionClient('head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            head_client.wait_for_server()
            #rospy.loginfo('...connected.')

            # Create a single-point head trajectory with the head_goal as the end-point
            head_trajectory = JointTrajectory()
            head_trajectory.joint_names = head_joints
            head_trajectory.points.append(JointTrajectoryPoint())
            head_trajectory.points[0].positions = head_goal
            head_trajectory.points[0].velocities = [0.0 for i in head_joints]
            head_trajectory.points[0].accelerations = [0.0 for i in head_joints]
            head_trajectory.points[0].time_from_start = rospy.Duration(2.0)

            # Send the trajectory to the head action server
            #rospy.loginfo('Moving the head to goal position...')

            head_goal = FollowJointTrajectoryGoal()
            head_goal.trajectory = head_trajectory
            head_goal.goal_time_tolerance = rospy.Duration(0.0)

            # Send the goal
            head_client.send_goal(head_goal)

            # Wait for up to 5 seconds for the motion to complete
            head_client.wait_for_result(rospy.Duration(2.0))
            
            rospy.loginfo('... to the left - head up')
        
            # step 3
        
            head_goal = [-1.0, 0.0]

            # Connect to the head trajectory action server
            #rospy.loginfo('Waiting for head trajectory controller...')
            head_client = actionlib.SimpleActionClient('head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            head_client.wait_for_server()
            #rospy.loginfo('...connected.')

            # Create a single-point head trajectory with the head_goal as the end-point
            head_trajectory = JointTrajectory()
            head_trajectory.joint_names = head_joints
            head_trajectory.points.append(JointTrajectoryPoint())
            head_trajectory.points[0].positions = head_goal
            head_trajectory.points[0].velocities = [0.0 for i in head_joints]
            head_trajectory.points[0].accelerations = [0.0 for i in head_joints]
            head_trajectory.points[0].time_from_start = rospy.Duration(2.0)

            # Send the trajectory to the head action server
            #rospy.loginfo('Moving the head to goal position...')

            head_goal = FollowJointTrajectoryGoal()
            head_goal.trajectory = head_trajectory
            head_goal.goal_time_tolerance = rospy.Duration(0.0)

            # Send the goal
            head_client.send_goal(head_goal)

            # Wait for up to 5 seconds for the motion to complete
            head_client.wait_for_result(rospy.Duration(2.0))
        
            rospy.loginfo('... to the right')
        
            # Set a goal configuration for the head - step 1
            head_goal = [-1.0, 0.5]

            # Connect to the head trajectory action server
            #rospy.loginfo('Waiting for head trajectory controller...')
            head_client = actionlib.SimpleActionClient('head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            head_client.wait_for_server()
            #rospy.loginfo('...connected.')

            # Create a single-point head trajectory with the head_goal as the end-point
            head_trajectory = JointTrajectory()
            head_trajectory.joint_names = head_joints
            head_trajectory.points.append(JointTrajectoryPoint())
            head_trajectory.points[0].positions = head_goal
            head_trajectory.points[0].velocities = [0.0 for i in head_joints]
            head_trajectory.points[0].accelerations = [0.0 for i in head_joints]
            head_trajectory.points[0].time_from_start = rospy.Duration(2.0)

            # Send the trajectory to the head action server
            #rospy.loginfo('Moving the head to goal position...')
    
            head_goal = FollowJointTrajectoryGoal()
            head_goal.trajectory = head_trajectory
            head_goal.goal_time_tolerance = rospy.Duration(0.0)

            # Send the goal
            head_client.send_goal(head_goal)

            # Wait for up to 5 seconds for the motion to complete
            head_client.wait_for_result(rospy.Duration(2.0))

            rospy.loginfo('... to the right - head down')
        
            # Set a goal configuration for the head - step 1
            head_goal = [-1.0, -0.5]

            # Connect to the head trajectory action server
            #rospy.loginfo('Waiting for head trajectory controller...')
            head_client = actionlib.SimpleActionClient('head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            head_client.wait_for_server()
            #rospy.loginfo('...connected.')

            # Create a single-point head trajectory with the head_goal as the end-point
            head_trajectory = JointTrajectory()
            head_trajectory.joint_names = head_joints
            head_trajectory.points.append(JointTrajectoryPoint())
            head_trajectory.points[0].positions = head_goal
            head_trajectory.points[0].velocities = [0.0 for i in head_joints]
            head_trajectory.points[0].accelerations = [0.0 for i in head_joints]
            head_trajectory.points[0].time_from_start = rospy.Duration(2.0)

            # Send the trajectory to the head action server
            #rospy.loginfo('Moving the head to goal position...')

            head_goal = FollowJointTrajectoryGoal()
            head_goal.trajectory = head_trajectory
            head_goal.goal_time_tolerance = rospy.Duration(0.0)

            # Send the goal
            head_client.send_goal(head_goal)

            # Wait for up to 5 seconds for the motion to complete
            head_client.wait_for_result(rospy.Duration(2.0))

            rospy.loginfo('... to the right - head up')
            
            # Set a goal configuration for the head - step 1
            head_goal = [0.0, 0.0]

            # Connect to the head trajectory action server
            #rospy.loginfo('Waiting for head trajectory controller...')
            head_client = actionlib.SimpleActionClient('head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            head_client.wait_for_server()
            #rospy.loginfo('...connected.')

            # Create a single-point head trajectory with the head_goal as the end-point
            head_trajectory = JointTrajectory()
            head_trajectory.joint_names = head_joints
            head_trajectory.points.append(JointTrajectoryPoint())
            head_trajectory.points[0].positions = head_goal
            head_trajectory.points[0].velocities = [0.0 for i in head_joints]
            head_trajectory.points[0].accelerations = [0.0 for i in head_joints]
            head_trajectory.points[0].time_from_start = rospy.Duration(2.0)

            # Send the trajectory to the head action server
            #rospy.loginfo('Moving the head to goal position...')

            head_goal = FollowJointTrajectoryGoal()
            head_goal.trajectory = head_trajectory
            head_goal.goal_time_tolerance = rospy.Duration(0.0)

            # Send the goal
            head_client.send_goal(head_goal)

            # Wait for up to 5 seconds for the motion to complete
            head_client.wait_for_result(rospy.Duration(2.0))

            rospy.loginfo('... head to zero')
        
        
        
            # Step 2 - moving arm to the first position
       
            # Set a goal configuration for torso
            torso_goal = [0.28]

            # Set a goal configuration for the arm
            arm_goal1  = [1.3, 1.5, 1.0, -1.7, 0.0, -1.7, 0.0]
            
            # Connect to the torso trajectory action server
            rospy.loginfo('Waiting for torso trajectory controller ...')
            torso_client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            torso_client.wait_for_server()
            rospy.loginfo('... connected.')

            # Connect to the right arm trajectory action server
            rospy.loginfo('Waiting for arm trajectory controller...')
            arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            arm_client.wait_for_server()
            rospy.loginfo('...connected.')

            # Create a single point torso trajectory with the torso_goal as the end point
            torso_trajectory = JointTrajectory()
            torso_trajectory.joint_names = torso_joints
            torso_trajectory.points.append(JointTrajectoryPoint())
            torso_trajectory.points[0].positions = torso_goal
            torso_trajectory.points[0].velocities = [0.0 for i in torso_joints]
            torso_trajectory.points[0].accelerations = [0.0 for i in torso_joints]
            torso_trajectory.points[0].time_from_start = rospy.Duration(5.0)

            # Send the trajectory to the torso action server
            rospy.loginfo('Raising torso ...')

            # Create an empty trajectory goal
            torso_goal = FollowJointTrajectoryGoal()

            # Set the trajectory component to the goal trajectory created above
            torso_goal.trajectory = torso_trajectory

            # Specify zero tolerance for the execution time
            torso_goal.goal_time_tolerance = rospy.Duration(0.0)

            # Send the goal to the action server
            torso_client.send_goal(torso_goal)

           # Wait for 5 seconds for the motion to complete
            torso_client.wait_for_result(rospy.Duration(5.0))


            # Create a single-point arm trajectory with the arm_goal as the end-point
            arm_trajectory = JointTrajectory()
            arm_trajectory.joint_names = arm_joints
            arm_trajectory.points.append(JointTrajectoryPoint())
            arm_trajectory.points[0].positions = arm_goal1
            arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
            arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
            arm_trajectory.points[0].time_from_start = rospy.Duration(5.0)
        
            # Send the trajectory to the arm action server
            rospy.loginfo('Moving the arm to goal position...')

            # Create an empty trajectory goal
            arm_goal1 = FollowJointTrajectoryGoal()

            # Set the trajectory component to the goal trajectory created above
            arm_goal1.trajectory = arm_trajectory

            # Specify zero tolerance for the execution time
            arm_goal1.goal_time_tolerance = rospy.Duration(0.0)

            # Send the goal to the action server
            arm_client.send_goal(arm_goal1)
            
            if not sync:
            # Wait for up to 5 seconds for the motion to complete
                arm_client.wait_for_result(rospy.Duration(5.0))
        
                rospy.loginfo('... first position reached')
        
        
            # step 2
            # Set a goal configuration for torso
            torso_goal = [0.28]
            
            # Set a goal configuration for the arm
            arm_goal2  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
            # Connect to the torso trajectory action server
            #rospy.loginfo('Waiting for torso trajectory controller ...')
            torso_client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            torso_client.wait_for_server()
            #rospy.loginfo('... connected.')

            # Connect to the right arm trajectory action server
            #rospy.loginfo('Waiting for arm trajectory controller...')
            arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            arm_client.wait_for_server()
            #rospy.loginfo('...connected.')

            # Create a single point torso trajectory with the torso_goal as the end point
            torso_trajectory = JointTrajectory()
            torso_trajectory.joint_names = torso_joints
            torso_trajectory.points.append(JointTrajectoryPoint())
            torso_trajectory.points[0].positions = torso_goal
            torso_trajectory.points[0].velocities = [0.0 for i in torso_joints]
            torso_trajectory.points[0].accelerations = [0.0 for i in torso_joints]
            torso_trajectory.points[0].time_from_start = rospy.Duration(5.0)

            # Send the trajectory to the torso action server
            #rospy.loginfo('Raising torso ...')

            # Create an empty trajectory goal
            torso_goal = FollowJointTrajectoryGoal()

            # Set the trajectory component to the goal trajectory created above
            torso_goal.trajectory = torso_trajectory

            # Specify zero tolerance for the execution time
            torso_goal.goal_time_tolerance = rospy.Duration(0.0)

            # Send the goal to the action server
            torso_client.send_goal(torso_goal)

            # Wait for 5 seconds for the motion to complete
            torso_client.wait_for_result(rospy.Duration(5.0))
        
            # Create a single-point arm trajectory with the arm_goal as the end-point
            arm_trajectory = JointTrajectory()
            arm_trajectory.joint_names = arm_joints
            arm_trajectory.points.append(JointTrajectoryPoint())
            arm_trajectory.points[0].positions = arm_goal2
            arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
            arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
            arm_trajectory.points[0].time_from_start = rospy.Duration(5.0)

            # Send the trajectory to the arm action server
            #rospy.loginfo('Moving the arm to zero position...')
    
            # Create an empty trajectory goal
            arm_goal2 = FollowJointTrajectoryGoal()

            # Set the trajectory component to the goal trajectory created above
            arm_goal2.trajectory = arm_trajectory

            # Specify zero tolerance for the execution time
            arm_goal2.goal_time_tolerance = rospy.Duration(5.0)

            # Send the goal to the action server
            arm_client.send_goal(arm_goal2)

            if not sync:
                # Wait for up to 5 seconds for the motion to complete
                arm_client.wait_for_result(rospy.Duration(5.0))
        
            rospy.loginfo('... zero position set')
        
        
        
            # step 3
             # Set a goal configuration for torso
            torso_goal = [0.28]
            # Set a goal configuration for the arm
            arm_goal3  = [-1.5, 1.0, 0.0, -2.0, 0.0, -1.0, 0.0]

           # Connect to the torso trajectory action server
            #rospy.loginfo('Waiting for torso trajectory controller ...')
            torso_client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            torso_client.wait_for_server()
            # rospy.loginfo('... connected.')
            
            # Connect to the right arm trajectory action server
            #rospy.loginfo('Waiting for arm trajectory controller...')
            arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            arm_client.wait_for_server()
            #rospy.loginfo('...connected.')

            # Create a single point torso trajectory with the torso_goal as the end point
            torso_trajectory = JointTrajectory()
            torso_trajectory.joint_names = torso_joints
            torso_trajectory.points.append(JointTrajectoryPoint())
            torso_trajectory.points[0].positions = torso_goal
            torso_trajectory.points[0].velocities = [0.0 for i in torso_joints]
            torso_trajectory.points[0].accelerations = [0.0 for i in torso_joints]
            torso_trajectory.points[0].time_from_start = rospy.Duration(5.0)

            # Send the trajectory to the torso action server
            #rospy.loginfo('Raising torso ...')

            # Create an empty trajectory goal
            torso_goal = FollowJointTrajectoryGoal()

            # Set the trajectory component to the goal trajectory created above
            torso_goal.trajectory = torso_trajectory

            # Specify zero tolerance for the execution time
            torso_goal.goal_time_tolerance = rospy.Duration(0.0)

            # Send the goal to the action server
            torso_client.send_goal(torso_goal)
        
            # Wait for 5 seconds for the motion to complete
            torso_client.wait_for_result(rospy.Duration(5.0))

            # Create a single-point arm trajectory with the arm_goal as the end-point
            arm_trajectory = JointTrajectory()
            arm_trajectory.joint_names = arm_joints
            arm_trajectory.points.append(JointTrajectoryPoint())
            arm_trajectory.points[0].positions = arm_goal3
            arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
            arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
            arm_trajectory.points[0].time_from_start = rospy.Duration(5.0)

            # Send the trajectory to the arm action server
            #rospy.loginfo('Moving the arm to the third position...')

            # Create an empty trajectory goal
            arm_goal3 = FollowJointTrajectoryGoal()
            
            # Set the trajectory component to the goal trajectory created above
            arm_goal3.trajectory = arm_trajectory

            # Specify zero tolerance for the execution time
            arm_goal3.goal_time_tolerance = rospy.Duration(0.0)
            
            # Send the goal to the action server
            arm_client.send_goal(arm_goal3)

            if not sync:
                # Wait for up to 5 seconds for the motion to complete
                arm_client.wait_for_result(rospy.Duration(5.0))
        
            rospy.loginfo('... arm at the third position')
        
        
        
            # step 4
             # Set a goal configuration for torso
            torso_goal = [0.28]
            # Set a goal configuration for the arm
            arm_goal4  = [-1.5, 1.0, 0.0, -2.0, 0.0, 0.0, 0.0]
        
            # Connect to the torso trajectory action server
            #rospy.loginfo('Waiting for torso trajectory controller ...')
            torso_client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            torso_client.wait_for_server()
            #rospy.loginfo('... connected.')

            # Connect to the right arm trajectory action server
            #rospy.loginfo('Waiting for arm trajectory controller...')
            arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            arm_client.wait_for_server()
            #rospy.loginfo('...connected.')

            # Create a single point torso trajectory with the torso_goal as the end point
            torso_trajectory = JointTrajectory()
            torso_trajectory.joint_names = torso_joints
            torso_trajectory.points.append(JointTrajectoryPoint())
            torso_trajectory.points[0].positions = torso_goal
            torso_trajectory.points[0].velocities = [0.0 for i in torso_joints]
            torso_trajectory.points[0].accelerations = [0.0 for i in torso_joints]
            torso_trajectory.points[0].time_from_start = rospy.Duration(5.0)
        
            # Send the trajectory to the torso action server
            #rospy.loginfo('Raising torso ...')

            # Create an empty trajectory goal
            torso_goal = FollowJointTrajectoryGoal()

            # Set the trajectory component to the goal trajectory created above
            torso_goal.trajectory = torso_trajectory

            # Specify zero tolerance for the execution time
            torso_goal.goal_time_tolerance = rospy.Duration(0.0)

           # Send the goal to the action server
            torso_client.send_goal(torso_goal)
        
            # Wait for 5 seconds for the motion to complete
            torso_client.wait_for_result(rospy.Duration(5.0))

        
            # Create a single-point arm trajectory with the arm_goal as the end-point
            arm_trajectory = JointTrajectory()
            arm_trajectory.joint_names = arm_joints
            arm_trajectory.points.append(JointTrajectoryPoint())
            arm_trajectory.points[0].positions = arm_goal4
            arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
            arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
            arm_trajectory.points[0].time_from_start = rospy.Duration(5.0)

            # Send the trajectory to the arm action server
            rospy.loginfo('Waving arm ...')

           # Create an empty trajectory goal
            arm_goal4 = FollowJointTrajectoryGoal()

            # Set the trajectory component to the goal trajectory created above
            arm_goal4.trajectory = arm_trajectory

            # Specify zero tolerance for the execution time
            arm_goal4.goal_time_tolerance = rospy.Duration(5.0)

            # Send the goal to the action server
            arm_client.send_goal(arm_goal4)

            if not sync:
                # Wait for up to 5 seconds for the motion to complete
                arm_client.wait_for_result(rospy.Duration(5.0))
                
                rospy.loginfo('... HELLO !')
        
        
            # step 5
             # Set a goal configuration for torso
            torso_goal = [0.28]
            # Set a goal configuration for the arm
            arm_goal5  = [-1.5, 1.0, 0.0, -2.0, 0.0, -1.0, 0.0]
            
            # Connect to the torso trajectory action server
            #rospy.loginfo('Waiting for torso trajectory controller ...')
            torso_client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            torso_client.wait_for_server()
            #rospy.loginfo('... connected.')

            # Connect to the right arm trajectory action server
            #rospy.loginfo('Waiting for arm trajectory controller...')
            arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            arm_client.wait_for_server()
            #rospy.loginfo('...connected.')

            # Create a single point torso trajectory with the torso_goal as the end point
            torso_trajectory = JointTrajectory()
            torso_trajectory.joint_names = torso_joints
            torso_trajectory.points.append(JointTrajectoryPoint())
            torso_trajectory.points[0].positions = torso_goal
            torso_trajectory.points[0].velocities = [0.0 for i in torso_joints]
            torso_trajectory.points[0].accelerations = [0.0 for i in torso_joints]
            torso_trajectory.points[0].time_from_start = rospy.Duration(5.0)
    
            # Send the trajectory to the torso action server
            #rospy.loginfo('Raising torso ...')
    
            # Create an empty trajectory goal
            torso_goal = FollowJointTrajectoryGoal()
    
            # Set the trajectory component to the goal trajectory created above
            torso_goal.trajectory = torso_trajectory
    
            # Specify zero tolerance for the execution time
            torso_goal.goal_time_tolerance = rospy.Duration(0.0)
    
            # Send the goal to the action server
            torso_client.send_goal(torso_goal)
    
            # Wait for 5 seconds for the motion to complete
            torso_client.wait_for_result(rospy.Duration(5.0))

            
            # Create a single-point arm trajectory with the arm_goal as the end-point
            arm_trajectory = JointTrajectory()
            arm_trajectory.joint_names = arm_joints
            arm_trajectory.points.append(JointTrajectoryPoint())
            arm_trajectory.points[0].positions = arm_goal5
            arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
            arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
            arm_trajectory.points[0].time_from_start = rospy.Duration(5.0)
    
            # Send the trajectory to the arm action server
           #rospy.loginfo('Waving arm ...')
    
            # Create an empty trajectory goal
            arm_goal5 = FollowJointTrajectoryGoal()
    
            # Set the trajectory component to the goal trajectory created above
            arm_goal5.trajectory = arm_trajectory
    
            # Specify zero tolerance for the execution time
            arm_goal5.goal_time_tolerance = rospy.Duration(5.0)
    
            # Send the goal to the action server
            arm_client.send_goal(arm_goal5)
    
            if not sync:
                # Wait for up to 5 seconds for the motion to complete
                arm_client.wait_for_result(rospy.Duration(5.0))
    
            rospy.loginfo('... WELCOME TO CMS !')
            
            
            
            # step 6
             # Set a goal configuration for torso
            torso_goal = [0.28]
            # Set a goal configuration for the arm
            arm_goal6  = [-1.5, 1.0, 0.0, -2.0, 0.0, 0.0, 0.0]
            
            # Connect to the torso trajectory action server
            #rospy.loginfo('Waiting for torso trajectory controller ...')
            torso_client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            torso_client.wait_for_server()
            #rospy.loginfo('... connected.')
    
            # Connect to the right arm trajectory action server
            #rospy.loginfo('Waiting for arm trajectory controller...')
            arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            arm_client.wait_for_server()
            #rospy.loginfo('...connected.')
    
            # Create a single point torso trajectory with the torso_goal as the end point
            torso_trajectory = JointTrajectory()
            torso_trajectory.joint_names = torso_joints
            torso_trajectory.points.append(JointTrajectoryPoint())
            torso_trajectory.points[0].positions = torso_goal
            torso_trajectory.points[0].velocities = [0.0 for i in torso_joints]
            torso_trajectory.points[0].accelerations = [0.0 for i in torso_joints]
            torso_trajectory.points[0].time_from_start = rospy.Duration(5.0)

            # Send the trajectory to the torso action server
            #rospy.loginfo('Raising torso ...')
    
            # Create an empty trajectory goal
            torso_goal = FollowJointTrajectoryGoal()
    
            # Set the trajectory component to the goal trajectory created above
            torso_goal.trajectory = torso_trajectory
    
            # Specify zero tolerance for the execution time
            torso_goal.goal_time_tolerance = rospy.Duration(0.0)
    
            # Send the goal to the action server
            torso_client.send_goal(torso_goal)
    
            # Wait for 5 seconds for the motion to complete
            torso_client.wait_for_result(rospy.Duration(5.0))
    
            
            # Create a single-point arm trajectory with the arm_goal as the end-point
            arm_trajectory = JointTrajectory()
            arm_trajectory.joint_names = arm_joints
            arm_trajectory.points.append(JointTrajectoryPoint())
            arm_trajectory.points[0].positions = arm_goal6
            arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
            arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
            arm_trajectory.points[0].time_from_start = rospy.Duration(5.0)
    
            # Send the trajectory to the arm action server
            #rospy.loginfo('Waving arm ...')
    
            # Create an empty trajectory goal
            arm_goal6 = FollowJointTrajectoryGoal()
    
            # Set the trajectory component to the goal trajectory created above
            arm_goal6.trajectory = arm_trajectory
    
            # Specify zero tolerance for the execution time
            arm_goal6.goal_time_tolerance = rospy.Duration(0.0)
    
            # Send the goal to the action server
            arm_client.send_goal(arm_goal6)
    
            if not sync:
                # Wait for up to 5 seconds for the motion to complete
                arm_client.wait_for_result(rospy.Duration(5.0))
    
            rospy.loginfo('... MY NAME IS FETCH !')
            
            
            
            # step 7
             # Set a goal configuration for torso
            torso_goal = [0.28]
            # Set a goal configuration for the arm
            arm_goal7  = [-1.5, 1.0, 0.0, -2.0, 0.0, -1.0, 0.0]
            
            # Connect to the torso trajectory action server
            #rospy.loginfo('Waiting for torso trajectory controller ...')
            torso_client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            torso_client.wait_for_server()
            #rospy.loginfo('... connected.')
    
            # Connect to the right arm trajectory action server
            #rospy.loginfo('Waiting for arm trajectory controller...')
            arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            arm_client.wait_for_server()
            #rospy.loginfo('...connected.')
    
            # Create a single point torso trajectory with the torso_goal as the end point
            torso_trajectory = JointTrajectory()
            torso_trajectory.joint_names = torso_joints
            torso_trajectory.points.append(JointTrajectoryPoint())
            torso_trajectory.points[0].positions = torso_goal
            torso_trajectory.points[0].velocities = [0.0 for i in torso_joints]
            torso_trajectory.points[0].accelerations = [0.0 for i in torso_joints]
            torso_trajectory.points[0].time_from_start = rospy.Duration(5.0)
    
            # Send the trajectory to the torso action server
            #rospy.loginfo('Raising torso ...')
    
            # Create an empty trajectory goal
            torso_goal = FollowJointTrajectoryGoal()
    
            # Set the trajectory component to the goal trajectory created above
            torso_goal.trajectory = torso_trajectory
    
            # Specify zero tolerance for the execution time
            torso_goal.goal_time_tolerance = rospy.Duration(0.0)
    
            # Send the goal to the action server
            torso_client.send_goal(torso_goal)
    
            # Wait for 5 seconds for the motion to complete
            torso_client.wait_for_result(rospy.Duration(5.0))

            
            # Create a single-point arm trajectory with the arm_goal as the end-point
            arm_trajectory = JointTrajectory()
            arm_trajectory.joint_names = arm_joints
            arm_trajectory.points.append(JointTrajectoryPoint())
            arm_trajectory.points[0].positions = arm_goal7
            arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
            arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
            arm_trajectory.points[0].time_from_start = rospy.Duration(5.0)
    
            # Send the trajectory to the arm action server
            #rospy.loginfo('Waving arm ...')
    
            # Create an empty trajectory goal
            arm_goal7 = FollowJointTrajectoryGoal()
    
            # Set the trajectory component to the goal trajectory created above
            arm_goal7.trajectory = arm_trajectory
    
            # Specify zero tolerance for the execution time
            arm_goal7.goal_time_tolerance = rospy.Duration(0.0)
    
            # Send the goal to the action server
            arm_client.send_goal(arm_goal7)
    
            if not sync:
                # Wait for up to 5 seconds for the motion to complete
                arm_client.wait_for_result(rospy.Duration(5.0))
    
            rospy.loginfo('... WELCOME TO VIRGINIA TECH !')
            
            
            # step 8
             # Set a goal configuration for torso
            torso_goal = [0.28]
            # Set a goal configuration for the arm
            arm_goal8 = [-1.5, 1.0, 0.0, -2.0, 0.0, -0.7, 0.0]
            
            # Connect to the torso trajectory action server
            #rospy.loginfo('Waiting for torso trajectory controller ...')
            torso_client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            torso_client.wait_for_server()
            #rospy.loginfo('... connected.')
    
            # Connect to the right arm trajectory action server
            #rospy.loginfo('Waiting for arm trajectory controller...')
            arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            arm_client.wait_for_server()
            #rospy.loginfo('...connected.')
    
            # Create a single point torso trajectory with the torso_goal as the end point
            torso_trajectory = JointTrajectory()
            torso_trajectory.joint_names = torso_joints
            torso_trajectory.points.append(JointTrajectoryPoint())
            torso_trajectory.points[0].positions = torso_goal
            torso_trajectory.points[0].velocities = [0.0 for i in torso_joints]
            torso_trajectory.points[0].accelerations = [0.0 for i in torso_joints]
            torso_trajectory.points[0].time_from_start = rospy.Duration(5.0)
    
            # Send the trajectory to the torso action server
            #rospy.loginfo('Raising torso ...')
    
            # Create an empty trajectory goal
            torso_goal = FollowJointTrajectoryGoal()
    
            # Set the trajectory component to the goal trajectory created above
            torso_goal.trajectory = torso_trajectory
    
            # Specify zero tolerance for the execution time
            torso_goal.goal_time_tolerance = rospy.Duration(0.0)
    
            # Send the goal to the action server
            torso_client.send_goal(torso_goal)
    
            # Wait for 5 seconds for the motion to complete
            torso_client.wait_for_result(rospy.Duration(5.0))
    
            
            # Create a single-point arm trajectory with the arm_goal as the end-point
            arm_trajectory = JointTrajectory()
            arm_trajectory.joint_names = arm_joints
            arm_trajectory.points.append(JointTrajectoryPoint())
            arm_trajectory.points[0].positions = arm_goal8
            arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
            arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
            arm_trajectory.points[0].time_from_start = rospy.Duration(5.0)
    
            # Send the trajectory to the arm action server
            #rospy.loginfo('Waving arm ...')
    
            # Create an empty trajectory goal
            arm_goal8 = FollowJointTrajectoryGoal()
    
            # Set the trajectory component to the goal trajectory created above
            arm_goal8.trajectory = arm_trajectory
    
            # Specify zero tolerance for the execution time
            arm_goal8.goal_time_tolerance = rospy.Duration(0.0)
    
            # Send the goal to the action server
            arm_client.send_goal(arm_goal8)
    
            if not sync:
                # Wait for up to 5 seconds for the motion to complete
                arm_client.wait_for_result(rospy.Duration(5.0))
    
            rospy.loginfo('... NICE TO SEE YOU HERE !')
            
            
            # step 9
             # Set a goal configuration for torso
            torso_goal = [0.28]
            # Set a goal configuration for the arm
            arm_goal9 = [-1.5, 1.0, 0.0, -2.0, 0.0, -0.7, 1.57]
            
            # Connect to the torso trajectory action server
            #rospy.loginfo('Waiting for torso trajectory controller ...')
            torso_client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            torso_client.wait_for_server()
            #rospy.loginfo('... connected.')
    
            # Connect to the right arm trajectory action server
            #rospy.loginfo('Waiting for arm trajectory controller...')
            arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            arm_client.wait_for_server()
            #rospy.loginfo('...connected.')
    
            # Create a single point torso trajectory with the torso_goal as the end point
            torso_trajectory = JointTrajectory()
            torso_trajectory.joint_names = torso_joints
            torso_trajectory.points.append(JointTrajectoryPoint())
            torso_trajectory.points[0].positions = torso_goal
            torso_trajectory.points[0].velocities = [0.0 for i in torso_joints]
            torso_trajectory.points[0].accelerations = [0.0 for i in torso_joints]
            torso_trajectory.points[0].time_from_start = rospy.Duration(5.0)

            # Send the trajectory to the torso action server
            #rospy.loginfo('Raising torso ...')
    
            # Create an empty trajectory goal
            torso_goal = FollowJointTrajectoryGoal()
    
            # Set the trajectory component to the goal trajectory created above
            torso_goal.trajectory = torso_trajectory
    
            # Specify zero tolerance for the execution time
            torso_goal.goal_time_tolerance = rospy.Duration(0.0)
    
            # Send the goal to the action server
            torso_client.send_goal(torso_goal)
    
            # Wait for 5 seconds for the motion to complete
            torso_client.wait_for_result(rospy.Duration(5.0))
    
            
            # Create a single-point arm trajectory with the arm_goal as the end-point
            arm_trajectory = JointTrajectory()
            arm_trajectory.joint_names = arm_joints
            arm_trajectory.points.append(JointTrajectoryPoint())
            arm_trajectory.points[0].positions = arm_goal9
            arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
            arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
            arm_trajectory.points[0].time_from_start = rospy.Duration(5.0)
    
            # Send the trajectory to the arm action server
            #rospy.loginfo('Waving arm ...')
    
            # Create an empty trajectory goal
            arm_goal9 = FollowJointTrajectoryGoal()
    
            # Set the trajectory component to the goal trajectory created above
            arm_goal9.trajectory = arm_trajectory
    
            # Specify zero tolerance for the execution time
            arm_goal9.goal_time_tolerance = rospy.Duration(0.0)
    
            # Send the goal to the action server
            arm_client.send_goal(arm_goal9)
    
            if not sync:
                # Wait for up to 5 seconds for the motion to complete
                arm_client.wait_for_result(rospy.Duration(5.0))
    
            rospy.loginfo('... WISH YOU HAVE A GREAT DAY !')
            
            time.sleep(10)
            
            
            ## Gripper open and close
            # step 9 - gripper
    
    
              
            
            
            # step 10
            # raising torso
            torso_goal = [0.4]
    
            # Set a goal configuration for the arm
            arm_goal10  = [-1.5, 1.0, 0.0, -2.0, 0.0, -0.7, 1.57]
            
            # Connect to the torso trajectory action server
            #rospy.loginfo('Waiting for torso trajectory controller ...')
            torso_client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            torso_client.wait_for_server()
            #rospy.loginfo('... connected.')
    
            # Connect to the right arm trajectory action server
            #rospy.loginfo('Waiting for arm trajectory controller...')
            arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            arm_client.wait_for_server()
            #rospy.loginfo('...connected.')
            
            # Create a single point torso trajectory with the torso_goal as the end point
            torso_trajectory = JointTrajectory()
            torso_trajectory.joint_names = torso_joints
            torso_trajectory.points.append(JointTrajectoryPoint())
            torso_trajectory.points[0].positions = torso_goal
            torso_trajectory.points[0].velocities = [0.0 for i in torso_joints]
            torso_trajectory.points[0].accelerations = [0.0 for i in torso_joints]
            torso_trajectory.points[0].time_from_start = rospy.Duration(3.0)
    
            # Send the trajectory to the torso action server
            #rospy.loginfo('Raising torso ...')
    
            # Create an empty trajectory goal
            torso_goal = FollowJointTrajectoryGoal()
    
            # Set the trajectory component to the goal trajectory created above
            torso_goal.trajectory = torso_trajectory
    
            # Specify zero tolerance for the execution time
            torso_goal.goal_time_tolerance = rospy.Duration(0.0)
    
            # Send the goal to the action server
            torso_client.send_goal(torso_goal)
    
            # Wait for 5 seconds for the motion to complete
            torso_client.wait_for_result(rospy.Duration(3.0))
            
            # Create a single-point arm trajectory with the arm_goal as the end-point
            arm_trajectory = JointTrajectory()
            arm_trajectory.joint_names = arm_joints
            arm_trajectory.points.append(JointTrajectoryPoint())
            arm_trajectory.points[0].positions = arm_goal10
            arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
            arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
            arm_trajectory.points[0].time_from_start = rospy.Duration(5.0)
    
            # Send the trajectory to the arm action server
            #rospy.loginfo('Waving arm ...')
    
            # Create an empty trajectory goal
            arm_goal10 = FollowJointTrajectoryGoal()
    
            # Set the trajectory component to the goal trajectory created above
            arm_goal10.trajectory = arm_trajectory
    
            # Specify zero tolerance for the execution time
            arm_goal10.goal_time_tolerance = rospy.Duration(0.0)
    
            # Send the goal to the action server
            arm_client.send_goal(arm_goal10)
    
    
            rospy.loginfo('... go up')
            
            
            # step 11
            # raising torso
            torso_goal = [0.1]

            # Set a goal configuration for the arm
            arm_goal11  = [-1.5, 1.0, 0.0, -2.0, 0.0, -0.7, 1.57]
            
            # Connect to the torso trajectory action server
            #rospy.loginfo('Waiting for torso trajectory controller ...')
            torso_client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            torso_client.wait_for_server()
            #rospy.loginfo('... connected.')
    
            # Connect to the right arm trajectory action server
            #rospy.loginfo('Waiting for arm trajectory controller...')
            arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            arm_client.wait_for_server()
            #rospy.loginfo('...connected.')
            
            # Create a single point torso trajectory with the torso_goal as the end point
            torso_trajectory = JointTrajectory()
            torso_trajectory.joint_names = torso_joints
            torso_trajectory.points.append(JointTrajectoryPoint())
            torso_trajectory.points[0].positions = torso_goal
            torso_trajectory.points[0].velocities = [0.0 for i in torso_joints]
            torso_trajectory.points[0].accelerations = [0.0 for i in torso_joints]
            torso_trajectory.points[0].time_from_start = rospy.Duration(3.0)
    
            # Send the trajectory to the torso action server
            #rospy.loginfo('Raising torso ...')
    
            # Create an empty trajectory goal
            torso_goal = FollowJointTrajectoryGoal()
    
            # Set the trajectory component to the goal trajectory created above
            torso_goal.trajectory = torso_trajectory
    
            # Specify zero tolerance for the execution time
            torso_goal.goal_time_tolerance = rospy.Duration(0.0)
    
            # Send the goal to the action server
            torso_client.send_goal(torso_goal)
    
            # Wait for 5 seconds for the motion to complete
            torso_client.wait_for_result(rospy.Duration(3.0))
            
            # Create a single-point arm trajectory with the arm_goal as the end-point
            arm_trajectory = JointTrajectory()
            arm_trajectory.joint_names = arm_joints
            arm_trajectory.points.append(JointTrajectoryPoint())
            arm_trajectory.points[0].positions = arm_goal11
            arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
            arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
            arm_trajectory.points[0].time_from_start = rospy.Duration(5.0)
    
            # Send the trajectory to the arm action server
            #rospy.loginfo('Waving arm ...')
    
            # Create an empty trajectory goal
            arm_goal11 = FollowJointTrajectoryGoal()
    
            # Set the trajectory component to the goal trajectory created above
            arm_goal11.trajectory = arm_trajectory
    
            # Specify zero tolerance for the execution time
            arm_goal11.goal_time_tolerance = rospy.Duration(0.0)
    
            # Send the goal to the action server
            arm_client.send_goal(arm_goal11)
    
    
            rospy.loginfo('... go down')
            
            # step 12
            # raising torso
            torso_goal = [0.28]
    
            # Set a goal configuration for the arm
            arm_goal12  = [-1.5, 1.0, 0.0, -2.0, 0.0, -0.7, 1.57]
            
            # Connect to the torso trajectory action server
            #rospy.loginfo('Waiting for torso trajectory controller ...')
            torso_client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            torso_client.wait_for_server()
            #rospy.loginfo('... connected.')
    
            # Connect to the right arm trajectory action server
            #rospy.loginfo('Waiting for arm trajectory controller...')
            arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            arm_client.wait_for_server()
            #rospy.loginfo('...connected.')
            
            # Create a single point torso trajectory with the torso_goal as the end point
            torso_trajectory = JointTrajectory()
            torso_trajectory.joint_names = torso_joints
            torso_trajectory.points.append(JointTrajectoryPoint())
            torso_trajectory.points[0].positions = torso_goal
            torso_trajectory.points[0].velocities = [0.0 for i in torso_joints]
            torso_trajectory.points[0].accelerations = [0.0 for i in torso_joints]
            torso_trajectory.points[0].time_from_start = rospy.Duration(3.0)
    
            # Send the trajectory to the torso action server
            #rospy.loginfo('Raising torso ...')
    
            # Create an empty trajectory goal
            torso_goal = FollowJointTrajectoryGoal()
    
            # Set the trajectory component to the goal trajectory created above
            torso_goal.trajectory = torso_trajectory
    
            # Specify zero tolerance for the execution time
            torso_goal.goal_time_tolerance = rospy.Duration(0.0)
    
            # Send the goal to the action server
            torso_client.send_goal(torso_goal)
    
            # Wait for 5 seconds for the motion to complete
            torso_client.wait_for_result(rospy.Duration(100.0))
            # Create a single-point arm trajectory with the arm_goal as the end-point
            arm_trajectory = JointTrajectory()
            arm_trajectory.joint_names = arm_joints
            arm_trajectory.points.append(JointTrajectoryPoint())
            arm_trajectory.points[0].positions = arm_goal12
            arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
            arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
            arm_trajectory.points[0].time_from_start = rospy.Duration(5.0)
    
            # Send the trajectory to the arm action server
            #rospy.loginfo('Waving arm ...')
    
            # Create an empty trajectory goal
            arm_goal12 = FollowJointTrajectoryGoal()
    
            # Set the trajectory component to the goal trajectory created above
            arm_goal12.trajectory = arm_trajectory
    
            # Specify zero tolerance for the execution time
            arm_goal12.goal_time_tolerance = rospy.Duration(0.0)
    
            # Send the goal to the action server
            arm_client.send_goal(arm_goal12)
    
    
            rospy.loginfo('... height adjustment')
            
            time.sleep(30)      # break time
              
        
            # step 13 reset
            rospy.loginfo('Back to resting position ...')
            # Set a goal configuration for the arm
            arm_goal  = [1.3, 1.5, 3.0, -1.7, 0.0, -1.7, 0.0, 0.0]
    
            # Set a goal configuration for torso
            torso_goal = [0.0]
    
            # Connect to the right arm trajectory action server
            #rospy.loginfo('Waiting for arm trajectory controller...')
            arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            arm_client.wait_for_server()
            #rospy.loginfo('...connected.')
    
            # Connect to the torso trajectory action server
            #rospy.loginfo('Waiting for torso trajectory controller ...')
            torso_client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            torso_client.wait_for_server()
            #rospy.loginfo('... connected.')
    
            # Create a single-point arm trajectory with the arm_goal as the end-point
            arm_trajectory = JointTrajectory()
            arm_trajectory.joint_names = arm_joints
            arm_trajectory.points.append(JointTrajectoryPoint())
            arm_trajectory.points[0].positions = arm_goal
            arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
            arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
            arm_trajectory.points[0].time_from_start = rospy.Duration(3.0)
    
            # Send the trajectory to the arm action server
            #rospy.loginfo('Moving the arm to goal position...')
    
            # Create an empty trajectory goal
            arm_goal = FollowJointTrajectoryGoal()
    
            # Set the trajectory component to the goal trajectory created above
            arm_goal.trajectory = arm_trajectory
    
            # Specify zero tolerance for the execution time
            arm_goal.goal_time_tolerance = rospy.Duration(0.0)
    
            # Send the goal to the action server
            arm_client.send_goal(arm_goal)
    
            if not sync:
                # Wait for up to 5 seconds for the motion to complete
                arm_client.wait_for_result(rospy.Duration(3.0))
            
            # Create a single point torso trajectory with the torso_goal as the end point
            torso_trajectory = JointTrajectory()
            torso_trajectory.joint_names = torso_joints
            torso_trajectory.points.append(JointTrajectoryPoint())
            torso_trajectory.points[0].positions = torso_goal
            torso_trajectory.points[0].velocities = [0.0 for i in torso_joints]
            torso_trajectory.points[0].accelerations = [0.0 for i in torso_joints]
            torso_trajectory.points[0].time_from_start = rospy.Duration(3.0)
    
            # Send the trajectory to the torso action server
            rospy.loginfo('Lowering torso ...')
    
            # Create an empty trajectory goal
            torso_goal = FollowJointTrajectoryGoal()
    
            # Set the trajectory component to the goal trajectory created above
            torso_goal.trajectory = torso_trajectory
    
            # Specify zero tolerance for the execution time
            torso_goal.goal_time_tolerance = rospy.Duration(0.0)
    
            # Send the goal to the action server
            torso_client.send_goal(torso_goal)
    
            # Wait for 5 seconds for the motion to complete
            torso_client.wait_for_result(rospy.Duration(5.0))
   
            rospy.loginfo('... RESTING DONE!')

            idx = idx + 1
            
            #break
            
if __name__ == '__main__':
    try:
        TrajectoryDemo()
    except rospy.ROSInterruptException:
        pass
