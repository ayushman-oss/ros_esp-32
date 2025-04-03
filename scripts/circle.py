#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__' :
    rospy.init_node("circle")
    rospy.loginfo("Started")
    pub = rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=10)
    rate = rospy.Rate(10)
    while  not rospy.is_shutdown():
            m=Twist()
            m.linear.x=2.0
            m.angular.z=1.0
            pub.publish(m)
            rate.sleep()

