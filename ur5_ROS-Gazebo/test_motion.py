#!/usr/bin/env python2

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import json
from ur5_notebook.msg import Tracker
from math import pi
from std_msgs.msg import String
from std_srvs.srv import Empty
from moveit_commander.conversions import pose_to_list
import time

class pose():
  def __init__(self, x, y, z, qx, qy, qz, qw):
	  self.x = x
	  self.y = y
	  self.z = z
	  self.qx = qx
	  self.qy = qy
	  self.qz = qz
	  self.qw = qw


def quat_to_mat(pose):
  xx = pose.qx*pose.qx;
  yy = pose.qy*pose.qy;
  zz = pose.qz*pose.qz;

  xy = pose.qx*pose.qy;
  xz = pose.qx*pose.qz;
  xw = pose.qx*pose.qw;
  yz = pose.qy*pose.qz;
  yw = pose.qy*pose.qw;
  zw = pose.qz*pose.qw;

  m00 = 1 - 2 * ( yy + zz );
  m01 =     2 * ( xy - zw );
  m02 =     2 * ( xz + yw );

  m10  =     2 * ( xy + zw );
  m11  = 1 - 2 * ( xx + zz );
  m12  =     2 * ( yz - xw );

  m20  =     2 * ( xz - yw );
  m21  =     2 * ( yz + xw );
  m22 = 1 - 2 * ( xx + yy );

  m03 = pose.x
  m13 = pose.y
  m23 = pose.z
  m30 = 0
  m31 = 0
  m32 = 0
  m33 = 1

  return np.array([[m00,m01,m02,m03],[m10,m11,m12,m13],[m20,m21,m22,m23],[m30,m31,m32,m33]]);

def mat_to_quat(m1):

  tr = m1[0,0] + m1[1,1] + m1[2,2]

  if (tr > 0):
    S = np.sqrt(tr+1.0) * 2
    quatw = 0.25 * S
    quatx = (m1[2,1] - m1[1,2]) / S
    quaty = (m1[0,2] - m1[2,0]) / S
    quatz = (m1[1,0] - m1[0,1]) / S

  elif (m1[0,0] > m1[1,1]) and (m1[0,0] > m1[2,2]):
    S = np.sqrt(1.0 + m1[0,0] - m1[1,1] - m1[2,2]) * 2
    quatw = (m1[2,1] - m1[1,2]) / S
    quatx = 0.25 * S
    quaty = (m1[0,1] + m1[1,0]) / S
    quatz = (m1[0,2] + m1[2,0]) / S
 
  elif (m1[1,1] > m1[2,2]):
    S = np.sqrt(1.0 + m1[1,1] - m1[0,0] - m1[2,2]) * 2
    quatw = (m1[0,2] - m1[2,0]) / S
    quatx = (m1[0,1] + m1[1,0]) / S
    quaty = 0.25 * S
    quatz = (m1[1,2] + m1[2,1]) / S
  else:
    S = np.sqrt(1.0 + m1[2,2] - m1[0,0] - m1[1,1]) * 2
    quatw = (m1[1,0] - m1[0,1]) / S
    quatx = (m1[0,2] + m1[2,0]) / S
    quaty = (m1[1,2] + m1[2,1]) / S
    quatz = 0.25 * S

  result = pose(m1[0,3],m1[1,3],m1[2,3],quatx,quaty,quatz,quatw)

  return result

def go_home(data):
	# if gripper is on turn it off
	global gripper_on
	if (gripper_on==True):
		tracker.flag2 = 0
		cxy_pub.publish(tracker)
		gripper_on = False;

	##  Joint target planning
	default_joint_states = group.get_current_joint_values()
	default_joint_states[0] = -1.57691
	default_joint_states[1] = -1.71667
	default_joint_states[2] = 1.79266
	default_joint_states[3] = -1.67721
	default_joint_states[4] = -1.5705
	default_joint_states[5] = 0.0

	group.set_joint_value_target(default_joint_states)

	# Set the internal state to the current state
	group.set_start_state_to_current_state()
	plan = group.plan()

	group.execute(plan)
	gazebo_physics('gazebo/pause_physics')
	sim_status.publish('home')

def gazebo_physics(service):

	rospy.wait_for_service(service)
	try:
		service_call = rospy.ServiceProxy(service, Empty)
		resp = service_call()
		print(resp)
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)

def simulate(data):

	print(data.data)

	global gripper_on

	gazebo_physics('gazebo/unpause_physics')

	wpose = group.get_current_pose(end_effector_link).pose

	path_file = open('/var/path.json')
	path = json.load(path_file)
	for keyframe in path['keyframes']:
		if (keyframe['object_poses']):
			if (keyframe['semantics'] == 'initial'):	#First keyframe should provide the initial positions of the parts
				print('initial keyframe')
			else:
				if (keyframe['semantics'] == 'grasp_a') or (keyframe['semantics'] == 'grasp_b'):	#if the keyframe requires grasping get the grasp pose and open the gripper
					position = keyframe['object_poses'][path['objects'][keyframe['moving_part']-1]['name']]['position']
					orientation = keyframe['object_poses'][path['objects'][keyframe['moving_part']-1]['name']]['orientation']

					waypoints = []

					wpose.position.x -= 0.1  # Second move forward/backwards in (x)
					waypoints.append(copy.deepcopy(wpose))

					# Get positions from json file!
					wpose.position.x = position['x']
					wpose.position.y = position['y']
					wpose.position.z = position['z']-1.015

					waypoints.append(copy.deepcopy(wpose))


					group.set_start_state_to_current_state()
					(plan, fraction) = group.compute_cartesian_path(
					                                   waypoints,   # waypoints to follow
					                                   0.1,        # eef_step
					                                   0)         # jump_threshold
					group.execute(plan, wait=True);

					#OPEN GRIPPER!
					tracker.flag2 = 1
					cxy_pub.publish(tracker)
					time.sleep(1)	#wait 1 sec to make sure object is grasped
					gripper_on = True;

					# calculate grasp pose
					object_start_pose = quat_to_mat(pose(position['x'], position['y'], position['z']-1.015, orientation['x'], orientation['y'], orientation['z'], orientation['w']))
					eef_start_pose = quat_to_mat(pose(position['x'], position['y'], position['z']-1.015, 0.5,0.5,-0.5,0.5))
					object_start_pose_inv = np.linalg.inv(object_start_pose)
					grasp_pose = np.matmul(object_start_pose_inv, eef_start_pose)	#maybe the grasp pose needs to be global var


				elif (keyframe['semantics'] == 'place_a') or (keyframe['semantics'] == 'place_b'):
					position = keyframe['object_poses'][path['objects'][keyframe['moving_part']-1]['name']]['position']
					orientation = keyframe['object_poses'][path['objects'][keyframe['moving_part']-1]['name']]['orientation']
					
					TV_pose = keyframe['object_poses']['tv_49']['position']
					TV_orient = keyframe['object_poses']['tv_49']['orientation']

					object_next_pose = quat_to_mat(pose(position['x'], position['y'], position['z'], orientation['x'], orientation['y'], orientation['z'], orientation['w']))
					TV_next_pose = quat_to_mat(pose(TV_pose['x'], TV_pose['y'], TV_pose['z'], TV_orient['x'], TV_orient['y'], TV_orient['z'], TV_orient['w']))

					TV_next_pose_inv = np.linalg.inv(TV_next_pose)

					object_tv = np.matmul(TV_next_pose_inv, object_next_pose)
					
					tv_new_pose = quat_to_mat(pose(TV_pose['x'], TV_pose['y'], TV_pose['z']-1.015, TV_orient['x'], TV_orient['y'], TV_orient['z'], TV_orient['w'])) #This can be replaced with the new values of the TV if it has been moved

					object_new = np.matmul(tv_new_pose, object_tv)

					
					eef_next_pose = np.matmul(object_new, grasp_pose)

					#TO DO
					#MOVE ROBOT TO robot_pose = mat_to_quat(eef_next_pose)
					wpose = group.get_current_pose(end_effector_link).pose

					robot_pose = mat_to_quat(eef_next_pose);

					wpose.position.x = robot_pose.x
					wpose.position.y = robot_pose.y
					wpose.position.z = robot_pose.z
					wpose.orientation.x = round(robot_pose.qx, 3)
					wpose.orientation.y = round(robot_pose.qy, 3)
					wpose.orientation.z = round(robot_pose.qz, 3)
					wpose.orientation.w = round(robot_pose.qw, 3)

					print(wpose)
					waypoints = []
					waypoints.append(copy.deepcopy(wpose))

					#group.set_pose_target(wpose, end_effector_link);
					#group.go();
					group.set_start_state_to_current_state()
					(plan, fraction) = group.compute_cartesian_path(
					                                   waypoints,   # waypoints to follow
					                                   0.1,        # eef_step
					                                   0)         # jump_threshold
					group.execute(plan, wait=True);

					#CLOSE GRIPPER AND MOVE ROBOT SLIGHTLY UP TO AVOID COLLISIONS

					tracker.flag2 = 0
					cxy_pub.publish(tracker)

					time.sleep(1)	#wait 1 sec to make sure object is ungrasped
					gripper_on = False;

					wpose.position.z += 0.1
					group.set_pose_target(wpose, end_effector_link);
					group.go();

					
					
				else:
					position = keyframe['object_poses'][path['objects'][keyframe['moving_part']-1]['name']]['position']
					orientation = keyframe['object_poses'][path['objects'][keyframe['moving_part']-1]['name']]['orientation']
					

					TV_pose = keyframe['object_poses']['tv_49']['position']
					TV_orient = keyframe['object_poses']['tv_49']['orientation']

					object_next_pose = quat_to_mat(pose(position['x'], position['y'], position['z'], orientation['x'], orientation['y'], orientation['z'], orientation['w']))
					TV_next_pose = quat_to_mat(pose(TV_pose['x'], TV_pose['y'], TV_pose['z'], TV_orient['x'], TV_orient['y'], TV_orient['z'], TV_orient['w']))

					TV_next_pose_inv = np.linalg.inv(TV_next_pose)

					object_tv = np.matmul(TV_next_pose_inv, object_next_pose)
					
					#This can be replaced with the new position of the TV in respect to the robot if it has been moved
					tv_new_pose = quat_to_mat(pose(TV_pose['x'], TV_pose['y'], TV_pose['z']-1.015, TV_orient['x'], TV_orient['y'], TV_orient['z'], TV_orient['w'])) 

					object_new = np.matmul(tv_new_pose, object_tv)

					
					eef_next_pose = np.matmul(object_new, grasp_pose)

					#TO DO
					#MOVE ROBOT TO robot_pose = mat_to_quat(eef_next_pose)
					wpose = group.get_current_pose(end_effector_link).pose

					robot_pose = mat_to_quat(eef_next_pose);

					wpose.position.x = robot_pose.x
					wpose.position.y = robot_pose.y
					wpose.position.z = robot_pose.z
					wpose.orientation.x = round(robot_pose.qx, 3)
					wpose.orientation.y = round(robot_pose.qy, 3)
					wpose.orientation.z = round(robot_pose.qz, 3)
					wpose.orientation.w = round(robot_pose.qw, 3)

					print(wpose)
					waypoints = []
					waypoints.append(copy.deepcopy(wpose))

					group.set_start_state_to_current_state()
					(plan, fraction) = group.compute_cartesian_path(
					                                   waypoints,   # waypoints to follow
					                                   0.1,        # eef_step
					                                   0)         # jump_threshold
					group.execute(plan, wait=True);
					
					#group.set_pose_target(wpose, end_effector_link);
					#group.go();
		else:
			print("Empty keyframe!!")

	path_file.close()

	# notify that the simulation ended
	sim_status.publish("finish")


## BEGIN_SUB_TUTORIAL setup
##
## First initialize `moveit_commander`_ and a `rospy`_ node:
rospy.init_node('assembly_simulation',anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)


group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)

group.set_goal_position_tolerance(0.01)
group.set_goal_orientation_tolerance(0.1)
group.set_planning_time(0.1)
group.set_max_acceleration_scaling_factor(.5)
group.set_max_velocity_scaling_factor(.5)

#Gripper publisher
gripper_on = False;

tracker = Tracker()
cxy_pub = rospy.Publisher('cxy1', Tracker, queue_size=1)

#Simulation publisher
sim_status = rospy.Publisher('simulation_status', String, queue_size=1)

end_effector_link = group.get_end_effector_link()

# Move robot to starting position
go_home('initial')

rospy.Subscriber("play_simulation", String, simulate)
rospy.Subscriber("go_home", String, go_home)
rospy.spin()

