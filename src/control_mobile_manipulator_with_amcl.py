#!/usr/bin/env python3

"""
Read the robot pose from \/amcl_pose topic and keep the robot moving with specific velocity till it reaches a specific point on the map
position: 
      x: 0.2402543946897043
      y: 0.852651713499374
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.6715313816479372
      w: 0.7409761153114266

      position: 
      x: 0.26168533778596453
      y: 0.44877357184858896
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.6690037734958856
      w: 0.7432590067051094


"""

#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist


class BasePose:
    def __init__(self):
        self.position = None
        # self.msg = PoseWithCovarianceStamped
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback)
        
    
    def callback(self, data):
        pos = data.pose.pose.position.y
        # self.position.x = pos.x
        # self.position.y = pos.y
        self.position = pos
        print(self.position)
    
    def get_base_pose(self):
        return self.position


# up
# data =  [0.0, -0.0015339808305725455, -1.5738643407821655, 1.2394565343856812, 0.607456386089325, 0.24526229100000002, -0.0015621944290406715, -1.5504853040116338, 1.2211845169080313, 0.5788869513162985, 0.34488140500000003, -0.0015904080275087975, -1.527106267241102, 1.2029124994303817, 0.550317516543272, 0.421211316, -0.0016186216259769235, -1.5037272304705702, 1.1846404819527319, 0.5217480817702456, 0.486307087, -0.0016468352244450495, -1.4803481937000385, 1.166368464475082, 0.4931786469972191, 0.545826742, -0.0016750488229131755, -1.4569691569295067, 1.1480964469974322, 0.4646092122241926, 0.605346398, -0.0017032624213813015, -1.433590120158975, 1.1298244295197826, 0.4360397774511661, 0.6648660540000001, -0.0017314760198494277, -1.4102110833884431, 1.1115524120421327, 0.4074703426781397, 0.7243857100000001, -0.0017596896183175537, -1.3868320466179114, 1.093280394564483, 0.3789009079051132, 0.783905366, -0.0017879032167856797, -1.3634530098473796, 1.0750083770868333, 0.35033147313208673, 0.849001136, -0.0018161168152538057, -1.3400739730768478, 1.0567363596091834, 0.32176203835906025, 0.925331047, -0.0018443304137219316, -1.316694936306316, 1.0384643421315336, 0.29319260358603383, 1.025031099, -0.0018725440121900576, -1.2933158995357843, 1.020192324653884, 0.2646231688130073, 1.27029339, -0.0019007576106581836, -1.2699368627652525, 1.0019203071762341, 0.2360537340399809]

#home
data = [0.0, -0.0015339808305725455, -1.2701361179351807, 1.023165225982666, 0.23623304069042206, 0.25301703000000003, -0.001580357939315339, -1.2951209516803035, 1.0403951278715782, 0.26654234925480946, 0.356466198, -0.0016267350480581324, -1.3201057854254263, 1.0576250297604906, 0.29685165781919687, 0.43584899400000005, -0.0016731121568009256, -1.3450906191705494, 1.0748549316494027, 0.3271609663835843, 0.503548329, -0.001719489265543719, -1.3700754529156722, 1.0920848335383149, 0.3574702749479717, 0.566692722, -0.0017658663742865124, -1.395060286660795, 1.1093147354272272, 0.38777958351235914, 0.6298371150000001, -0.0018122434830293058, -1.4200451204059181, 1.1265446373161394, 0.41808889207674654, 0.692981508, -0.0018586205917720993, -1.445029954151041, 1.1437745392050516, 0.448398200641134, 0.7561259010000001, -0.0019049977005148925, -1.4700147878961638, 1.161004441093964, 0.47870750920552135, 0.823154946, -0.0019513748092576861, -1.4949996216412866, 1.178234342982876, 0.5090168177699088, 0.9017517730000001, -0.0019977519180004793, -1.5199844553864095, 1.1954642448717883, 0.5393261263342962, 1.004641131, -0.002044129026743273, -1.5449692891315325, 1.2126941467607006, 0.5696354348986836, 1.257658161, -0.002090506135486066, -1.5699541228766554, 1.2299240486496128, 0.599944743463071]



from std_msgs.msg import Float64MultiArray
if __name__=="__main__":
    
    rospy.init_node('turtlebot3_teleop_1')
    rate = rospy.Rate(12)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=20)

    turtlebot3_model = rospy.get_param("model", "burger")

    status = 0

    # Arm Trajectory Msg
    pub_joints = rospy.Publisher('joint_trajectory_point', Float64MultiArray, queue_size=10)
    arm_trajectory_msg = Float64MultiArray()
    arm_trajectory_msg.data = data

    t1 = rospy.get_time()
    base_pose = BasePose()
    try:
        while not rospy.is_shutdown():
            print('0.0')
            t = base_pose.get_base_pose()
            print('0.2')
            print(t, type(t))

            #Publish Arm trajectory
            pub_joints.publish(arm_trajectory_msg)


            twist = Twist()
            # b_pose = base_pose.get_base_pose()
            # rospy.loginfo(b_pose)
            if t!=None:
                print('1st')
                if t < 2.5:
                    print('2nd')
                    control_linear_vel = 0.00
                else:
                    control_linear_vel = 0.00
            # control_linear_vel = 0.00
            # control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            # control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0  #control_angular_vel

            pub.publish(twist)
            rate.sleep()

    except:
        print('not working')

    rospy.spin()