#!/usr/bin/env python3
import rospy

if __name__ == '__main__':
    rospy.init_node('test_speed')
    t = rospy.get_rostime()
    rospy.loginfo("time: %d", t.to_sec())
    t1 = rospy.get_rostime()
    rospy.loginfo("time: %f", t1.to_sec()-t.to_sec())

# 1.6 ms delay between two log statements