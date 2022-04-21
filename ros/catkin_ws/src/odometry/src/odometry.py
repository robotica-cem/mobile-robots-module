#!/usr/bin/env python
""" Implement a node that calculates the odometry for the puzzlebot robot.
"""

import numpy as np
import rospy
import tf2_ros
from tf import transformations as trf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, TransformStamped, PoseWithCovariance,TwistWithCovariance
from std_msgs.msg import Float32

WHEEL_DISTANCE = 0.18 # From puzzlebot_gazebo/urdf/puzzle_bot.xacro
WHEEL_RADIUS = 0.05 # From puzzlebot_gazebo/urdf/parameters.xacro

class MyOdometryPublisher():
    def __init__(self):

        rospy.init_node('OdometryPublisher')

        # Subscribe to the wheel velocity topics
        rospy.Subscriber("/wl", Float32, self._wl_callback)
        rospy.Subscriber("/wr", Float32, self._wr_callback)
        self.wl = None
        self.wr = None
        
        # Publish to the odometry topic
        self.odom_pub = rospy.Publisher("/odometry", Odometry, queue_size=1)

        # Publish the simpler (estimated) state to separate topics
        self.x_pub = rospy.Publisher("/est_state/x", Float32, queue_size=1)
        self.y_pub = rospy.Publisher("/est_state/y", Float32, queue_size=1)
        self.th_pub = rospy.Publisher("/est_state/theta", Float32, queue_size=1)

        self.model_state = np.array([0.0, 0.0, 0.0])
        self.model_pose = PoseWithCovariance()
        self.model_twist = TwistWithCovariance()

        # Keep track of time between updates to the state of the robot
        self.current_time = rospy.get_time()
        self.last_time = self.current_time
        
        # For broadcasting transform from base_link to odom 
        self.br = tf2_ros.TransformBroadcaster() 

        self.rate = rospy.Rate(20)
        
    def _wl_callback(self, data):
        self.wl = data.data
    def _wr_callback(self, data):
        self.wr = data.data
        
        
    def main(self):

        f1 = WHEEL_RADIUS/ WHEEL_DISTANCE #To save some calculations
        f2 = 0.5*WHEEL_RADIUS #To save some calculations
        while not rospy.is_shutdown():
            self.rate.sleep()
            
            self.current_time = rospy.get_time()
            dt  = self.current_time - self.last_time
            self.last_time = self.current_time
            
            if (self.wl is not None) and (self.wr is not None):
                w = (self.wr-self.wl) *f1
                v = (self.wr+self.wl)*f2
                delta_th = w*dt
                th = self.model_state[0]
                self.model_state += np.array([delta_th,
                                              v*dt*np.cos(th+0.5*delta_th),
                                              v*dt*np.sin(th+0.5*delta_th)])
                self.model_pose.pose.position.x = self.model_state[1]
                self.model_pose.pose.position.y = self.model_state[2]
                
                self.model_pose.pose.orientation.w = np.cos(self.model_state[0]/2)
                self.model_pose.pose.orientation.z = np.sin(self.model_state[0]/2)

                self.model_twist.twist.linear.x = v
                self.model_twist.twist.angular.z = w
                

            # Publish the transform between the odometry frame (fixed) and the base_link frame

            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation = self.model_pose.pose.position
            t.transform.rotation = self.model_pose.pose.orientation
            self.br.sendTransform(t)

            
            # Publish the odometry message
            odom = Odometry()
            # Fill the message with your data
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "odom"
            # Set the position
            odom.pose = self.model_pose
            odom.twist = self.model_twist
            
            # publish the message
            self.odom_pub.publish(odom)


            # Publish the state
            self.x_pub.publish(self.model_state[1])
            self.y_pub.publish(self.model_state[2])
            self.th_pub.publish(self.model_state[0])
                    
if __name__ == '__main__':

    try:
        aux = MyOdometryPublisher()
        aux.main()

    except (rospy.ROSInterruptException, rospy.ROSException("topic was closed during publish()")):
        pass
