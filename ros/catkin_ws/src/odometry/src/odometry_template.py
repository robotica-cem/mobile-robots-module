#!/usr/bin/env python
""" Implements a "ground truth" odometry using the model state information from Gazebo.
"""

import numpy as np
import rospy
import tf2_ros
from tf import transformations as trf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3,Transform,TransformStamped,Pose,PoseStamped,PoseWithCovariance,Twist,TwistWithCovariance
from std_msgs.msg import Float64


class MyOdometryPublisher():
    def __init__(self):

        rospy.init_node('OdometryPublisher')

        # Subscribe to the wheel velocity topics
        rospy.Subscriber("/wl", Float64, self.callback)

        # Publish to the odometry topic
        self.odom_pub = rospy.Publisher("/odometry", Odometry, queue_size=1)

        # Publish the simpler (estimated) state to separate topics
        self.x_pub = rospy.Publisher("/est_state/x", Float64, queue_size=1)
        self.y_pub = rospy.Publisher("/est_state/y", Float64, queue_size=1)
        self.th_pub = rospy.Publisher("/est_state/theta", Float64, queue_size=1)

        self.initial_state = np.array([0.0, 0.0, 0.0])
        self.model_state = None
        self.model_twist = None

        # Keep track of time between updates to the state of the robot
        self.current_time = rospy.get_time()
        self.last_time = current_time
        
        # For broadcasting transform from base_link to odom 
        self.br = tf2_ros.TransformBroadcaster() 

        self.rate = rospy.Rate(20)
        

    def main(self):

        # If there's an object attached to the ee we want it to follow its trajectory
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.current_time = rospy.time()
            dt  = self.current_time - self.last_time

            #----------------------------------------------------------------------------
            # Your code here
            #----------------------------------------------------------------------------

            
            if self.initial_state is None:
                if self.model_state is not None:
                    # Get the initial pose 
                    t = TransformStamped()
                    t.header.stamp = rospy.Time.now()
                    t.header.frame_id = "world"
                    t.transform.translation = self.model_state.position
                    t.transform.rotation = self.model_state.orientation
                    self.brStatic.sendTransform(t)
                    self.initial_state = PoseStamped()
                    self.initial_state.header = t.header
                    self.initial_state.pose.position = t.transform.translation
                    self.initial_state.pose.orientation = t.transform.rotation
                    print("Set the initial state")
                    print(self.model_state)
                    
            else:
                if self.model_state is not None:
                    # Publish the state of the puzzlebot
                    t = TransformStamped()
                    t.header.stamp = rospy.Time.now()
                    t.header.frame_id = "odom_true"
                    t.child_frame_id = "base_link"

                    tt = transform_between_poses(self.initial_state.pose, self.model_state)
                    
                    t.transform.translation = tt.translation
                    t.transform.rotation = tt.rotation
                    self.br.sendTransform(t)
                    
                    # Publish the odometry message
                    odom = Odometry()
                    odom.header.stamp = t.header.stamp
                    odom.header.frame_id = "odom_true"
                    # Set the position
                    odom.pose.pose = Pose(t.transform.translation, t.transform.rotation)
                    # Set the velocity
                    odom.child_frame_id = "base_link"
                    vs = self.model_twist.linear # Velocity of puzzlebot in world frame
                    tr = Transform()
                    tr.rotation = self.model_state.orientation
                    vb = transform_vector(tr, (vs.x, vs.y, vs.z), inverse=True)
                    # Transform to base_link frame
                    odom.twist.twist = Twist(Vector3(*vb), Vector3(0,0,self.model_twist.angular.z))
                    
                    # publish the message
                    self.odom_pub.publish(odom)


                    # Publish the state
                    self.x_pub.publish(tt.translation.x)
                    self.y_pub.publish(tt.translation.y)
                    self.th_pub.publish(2*np.arccos(tt.rotation.w))
                    
                    
    def callback(self, data):
        aux_idx = data.name.index('puzzlebot')

        self.model_state = data.pose[aux_idx]
        self.model_twist = data.twist[aux_idx]
        
if __name__ == '__main__':

    try:
        aux = OdometryPublisher()
        aux.main()

    except (rospy.ROSInterruptException, rospy.ROSException("topic was closed during publish()")):
        pass
