

#!/usr/bin/env python

# wave.py: "Wave" the fetch gripper
import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface,PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import numpy as np
import tf
from shape_msgs.msg import SolidPrimitive

from geometry_msgs.msg import PointStamped

# Note: fetch_moveit_config move_group.launch must be running
# Safety!: Do NOT run this script near people or objects.
# Safety!: There is NO perception.
#          The ONLY objects the collision detection software is aware
#          of are itself & the floor.

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]


import rospy
import sys
import moveit_commander

# Initialize the MoveIt Python API
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('get_joint_limits_example', anonymous=True)

# Create a RobotCommander instance to get information about the robot's model
robot = moveit_commander.RobotCommander()
print(robot._r.get_joint_limits())
group_commander = moveit_commander.MoveGroupCommander('arm')

print()


    # gripper_frame = 'wrist_roll_link'
    # group_name = "arm_with_torso"
    # # # ### Figure out how to get Quaternion
    # # # # gripper_poses = [Pose(Point(0.96,0.0,0.78),
    # # # #                     Quaternion( 0.0,0.0,0.0,1.0))]
    # # #                     #   Quaternion(0.173, -0.693, -0.242, 0.657))] #,

    # # gripper_poses = [Pose(Point(0.53,-0.145,0.84+0.5),
    # #                     Quaternion( quats[0],quats[1], quats[2], quats[3]))]
    # # gripper_poses = [Pose(Point(0.75,0.056,1),
    # #                     Quaternion( quats[0],quats[1], quats[2], quats[3]))]
    # quats = tf.transformations.quaternion_from_euler(0.0, np.pi/10, 0.0)
    # gripper_poses = [Pose(Point(0.1-0.3,0.21,0.85),
    #                      Quaternion( quats[0],quats[1], quats[2], quats[3])),
    #                      Pose(Point(0.1-0.2,0.21,0.85),
    #                      Quaternion( quats[0],quats[1], quats[2], quats[3])),
    #                      Pose(Point(0.1-0.1,0.21,0.85),
    #                      Quaternion( quats[0],quats[1], quats[2], quats[3])),
    #                      Pose(Point(0.1,0.21,0.85),
    #                      Quaternion( quats[0],quats[1], quats[2], quats[3]))]
  



    # # gripper_poses =                 [Pose(Point(0.047, 0.545, 1.822),

    # ### Points in map frame
    # # gripper_poses = [Pose(Point(0.05, 0.55, 1.825),
    # #                       Quaternion(-0.274, -0.701, 0.173, 0.635))]

    # # # Construct a "pose_stamped" message as required by moveToPose
    
    
    # gripper_pose_stamped = PoseStamped()
    # gripper_pose_stamped.header.frame_id = '/map'

    # # while not rospy.is_shutdown():
    # for pose in gripper_poses:
    #     # Finish building the Pose_stamped message
    #     # If the message stamp is not current it could be ignored
    #     gripper_pose_stamped.header.stamp = rospy.Time.now()
    #     # Set the message pose
    #     gripper_pose_stamped.pose = pose

    #     # Move gripper frame to the pose specified
    #     move_group.moveToPose(gripper_pose_stamped, gripper_frame)
    #     result = move_group.get_move_action().get_result()
    #     # print("state:",move_group.get_move_action.get_state())

    #     if result:
    #         # Checking the MoveItErrorCode
    #         if result.error_code.val == MoveItErrorCodes.SUCCESS:
    #             rospy.loginfo("Hello there!")
    #         else:
    #             # If you get to this point please search for:
    #             # moveit_msgs/MoveItErrorCodes.msg
    #             rospy.logerr("Arm goal in state: %s",
    #                           move_group.get_move_action().get_state())
    #     else:
    #         rospy.logerr("MoveIt! failure no result returned.")

    # # This stops all arm movement goals
    # # It should be called when a program is exiting so movement stops
    # move_group.get_move_action().cancel_all_goals()

    # print("here: ",group.get_current_pose())


### Posible Pose of ee for grabing the cube
# pose: 
#   position: 
#     x: 0.526640434474
#     y: -0.14574979736
#     z: 0.875196833975
#   orientation: 
#     x: 0.0198842106793
#     y: 0.0279655075013
#     z: 0.0237472179888
#     w: 0.999128929715)


#### Grasp pose w.r.t base link
# pose: 
#   position: 
#     x: 0.530299766859
#     y: -0.144110748845
#     z: 0.909038515128
#   orientation: 
#     x: -0.000140213682545
#     y: 0.37787028227
#     z: 0.00440368078627
#     w: 0.925848064054

#### Cube pose
# x = 0.046
# y = 0.46
# z = 0.84