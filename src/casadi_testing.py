import casadi as cs
from urdf2casadi import urdfparser as u2c

urdf_path = "/home/akshaya/tb3om_noetic_ws/src/multi_mobile_manipulation_control/src/turtlebot_manipulator_model.urdf"
root_link = "base_link"
end_link = "gripper_link"
robot_parser = u2c.URDFparser()
robot_parser.from_file(urdf_path)
# Also supports .from_server for ros parameter server, or .from_string if you have the URDF as a string.
print(robot_parser.get_joint_info(root_link,end_link))
fk_dict = robot_parser.get_forward_kinematics(root_link, end_link)
print(fk_dict.keys())
# should give ['q', 'upper', 'lower', 'dual_quaternion_fk', 'joint_names', 'T_fk', 'joint_list', 'quaternion_fk']
forward_kinematics = fk_dict["T_fk"]