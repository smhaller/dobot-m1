#!/usr/bin/python
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import time

msg= JointTrajectory()
msg.joint_names=['dobot_m1_z_axis_joint', 'dobot_m1_axis_2_joint', 'dobot_m1_axis_3_joint', 'dobot_m1_axis_4_joint']

point=JointTrajectoryPoint()
Pos0=[0.23, -1.2, 1.5, 1.5]
Pos1=[0.2, 0, 0, 0]
Pos2=[0.15, 1.2, -1.5, -1.5]


def goTo(pub, pos):
  point.time_from_start=rospy.Duration(0.1)
  point.positions=pos
  msg.points=[point]
  pub.publish(msg)


def main():
  rospy.init_node("Demo-Dobot")
  rate=rospy.Rate(1)
  pub=rospy.Publisher('/dobby/joint_trajectory_controller/command', JointTrajectory, queue_size=1)

  raw_input("Press any key to continue with step")
  goTo(pub, Pos1)
  rate.sleep()

  raw_input("Press any key to continue with step")
  goTo(pub, Pos0)
  rate.sleep()

  raw_input("Press any key to continue with step")
  goTo(pub, Pos1)
  rate.sleep()

  raw_input("Press any key to continue with step")
  goTo(pub, Pos2)
  rate.sleep()

if __name__ == "__main__":
  main()


