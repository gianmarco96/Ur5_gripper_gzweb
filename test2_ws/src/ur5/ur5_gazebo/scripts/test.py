#!/usr/bin/env python3



# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import argparse

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
try:
  from math import pi, tau, dist, fabs, cos
except: # For Python 2 compatibility
  from math import pi, fabs, cos, sqrt
  tau = 2.0*pi
  def dist(p, q):
    return sqrt(sum((p_i - q_i)**2.0 for p_i, q_i in zip(p,q)))
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if the values in two lists are within a tolerance of each other.
  For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle 
  between the identical orientations q and -q is calculated correctly).
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
    x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
    # Euclidean distance
    d = dist((x1, y1, z1), (x0, y0, z0))
    # phi = angle between orientations
    cos_phi_half = fabs(qx0*qx1 + qy0*qy1 + qz0*qz1 + qw0*qw1)
    return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

  return True




moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

## Instantiate a `RobotCommander`_ object. Provides information such as the robot's kinematic model and the robot's current joint states
robot = moveit_commander.RobotCommander()

## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface for getting, setting, and updating the robot's internal understanding of the
## surrounding world:
# scene = moveit_commander.PlanningSceneInterface()

## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
## to a planning group (group of joints).  In this tutorial the group is the primary
## arm joints in the Panda robot, so we set the group's name to "panda_arm".
## If you are using a different robot, change this value to the name of your robot
## arm planning group.
## This interface can be used to plan and execute motions:
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

## Create a `DisplayTrajectory`_ ROS publisher which is used to display
## trajectories in Rviz:
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)



def go_to_joint_state():

  joint_goal = move_group.get_current_joint_values()
  print(joint_goal)
  joint_goal[0] = -pi/2
  joint_goal[1] = -1.80
  joint_goal[2] = 1.00
  joint_goal[3] = 0.0
  joint_goal[4] = 0.0
  joint_goal[5] = 0.0
  

  # The go command can be called with joint values, poses, or without any
  # parameters if you have already set the pose or joint target for the group
  move_group.go(joint_goal, wait=True)

  # Calling ``stop()`` ensures that there is no residual movement
  move_group.stop()

  ## END_SUB_TUTORIAL

  # For testing:
  current_joints = move_group.get_current_joint_values()
  return all_close(joint_goal, current_joints, 0.01)


def go_to_pose_goal(val):

  pose_goal = geometry_msgs.msg.Pose()
  q = quaternion_from_euler(3.141,0,0)
  pose_goal.orientation.x = q[0]
  pose_goal.orientation.y = q[1]
  pose_goal.orientation.z = q[2]
  pose_goal.orientation.w = q[3]
  pose_goal.position.x = 0.4
  pose_goal.position.y = 0.0
  pose_goal.position.z = val #0.35

  move_group.set_pose_target(pose_goal)

  ## Now, we call the planner to compute the plan and execute it.
  plan = move_group.go(wait=True)
  # Calling `stop()` ensures that there is no residual movement
  move_group.stop()
  # It is always good to clear your targets after planning with poses.
  # Note: there is no equivalent function for clear_joint_value_targets()
  move_group.clear_pose_targets()

  current_pose = move_group.get_current_pose().pose
  return all_close(pose_goal, current_pose, 0.01)


def plan_cartesian_path(scale=1):

  ## You can plan a Cartesian path directly by specifying a list of waypoints
  ## for the end-effector to go through. If executing  interactively in a
  ## Python shell, set scale = 1.0.
  ##
  waypoints = []

  wpose = move_group.get_current_pose().pose
  wpose.position.z -= scale * 0.1  # First move up (z)
  wpose.position.y += scale * 0.2  # and sideways (y)
  waypoints.append(copy.deepcopy(wpose))

  wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
  waypoints.append(copy.deepcopy(wpose))

  wpose.position.y -= scale * 0.1  # Third move sideways (y)
  waypoints.append(copy.deepcopy(wpose))

  # We want the Cartesian path to be interpolated at a resolution of 1 cm
  # which is why we will specify 0.01 as the eef_step in Cartesian
  # translation.  We will disable the jump threshold by setting it to 0.0,
  # ignoring the check for infeasible jumps in joint space, which is sufficient
  # for this tutorial.
  (plan, fraction) = move_group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold

  # Note: We are just planning, not asking move_group to actually move the robot yet:
  return plan, fraction

  ## END_SUB_TUTORIAL






def main():
  try:
    # go_to_joint_state()
    parser = argparse.ArgumentParser()
    parser.add_argument("--value", type=float, default="0.35",
                        help="0.35 pick position 0.50 move way position")
    args = parser.parse_args()
    go_to_pose_goal(args.value)
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

