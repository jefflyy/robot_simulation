import math
import time
import sys
import rospy
import signal
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from actionlib import SimpleActionClient
import actionlib
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Twist

def kb_interrupt_handler(signum, frame):
    print("quit")
    rospy.signal_shutdown("closed!")
    exit(0)


if __name__ == "__main__":

    rospy.init_node('plot_gps', anonymous=True)
    signal.signal(signal.SIGINT, kb_interrupt_handler)

    # first, use move_base to navigate to the platform
    actionlib_client = SimpleActionClient('move_base', MoveBaseAction)
    actionlib_client.wait_for_server()

    goal = MoveBaseGoal()

    goal.target_pose = PoseStamped()
    goal.target_pose.pose.position.x = -0.4
    goal.target_pose.pose.position.y = -0.78
    goal.target_pose.pose.orientation.w = 1
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    actionlib_client.send_goal(goal)
    ret = actionlib_client.wait_for_result(timeout=rospy.Duration(20))

    if ret:
        print("navigation success")

    # there may be some residual errors due to goal tolerance
    # the goal tolerance can be modified in move_base settings
    # you can make some small movements using visual servoing
    # here shows how to direct send velocity commands

    # raw_input("Continue?")

    # cmd_vel_pub = rospy.Publisher("/jetauto_controller/cmd_vel", Twist, queue_size=10)
    # small_vel = Twist()
    # small_vel.linear.x = 0.1
    # start_time = time.time()
    # while (time.time() - start_time) < 2:
    #     cmd_vel_pub.publish(small_vel)
    #     time.sleep(0.05)
    
    # cmd_vel_pub.publish(Twist())

    # second, use moveit_commander to close the gripper
    moveit_commander.roscpp_initialize(sys.argv)


    raw_input("Continue?")
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    gripper = moveit_commander.MoveGroupCommander("gripper")
    target_gripper_angle = 0.52
    target_gripper_joint_angle_list = [-target_gripper_angle, target_gripper_angle, -target_gripper_angle, target_gripper_angle, target_gripper_angle, target_gripper_angle]
    gripper.go(target_gripper_joint_angle_list)


    # third, use moveit_commander to lift the arm

    raw_input("Continue?")
    arm = moveit_commander.MoveGroupCommander("arm")
    # joint target or end effector pose target are both OK
    # please refer to API document
    lifted_joint_state = [0, 0, 0, 1.5708]
    arm.set_joint_value_target(lifted_joint_state)
    print("planning...")
    plan = arm.plan()
    # input("Execute?")
    arm.execute(plan, wait=True)