#!/usr/bin/env python  
import sys
import rospy
import math
import tf2_ros
import geometry_msgs.msg
from tf2_geometry_msgs import PointStamped

class Go2PointController:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.vel_pub = rospy.Publisher('/cmd_vel' , geometry_msgs.msg.Twist, queue_size=1 )
        self.rate = rospy.Rate(10.0)


    def go_to_point(self, x, y, frame):
        goal = PointStamped()
        goal.header.frame_id = frame
        goal.point.x = x
        goal.point.y = y
        goal.point.z = 0
        print(goal)
        
        while not rospy.is_shutdown():
            try:
                goal.header.stamp = rospy.Time.now()
                bot_goal = self.tf_buffer.transform(goal, 'base_link', timeout = rospy.Duration(1))

                msg = geometry_msgs.msg.Twist()

                #msg.angular.z = 
                #msg.linear.x = 
                # print(msg)
                self.vel_pub.publish(msg)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                #pass
                continue

        self.rate.sleep()        

if __name__ == '__main__':
    rospy.init_node('go_to_point_controller')
    g2p = Go2PointController()
    g2p.go_to_point(float(sys.argv[1]), float(sys.argv[2]), sys.argv[3])
    


