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
    def __init__(self, variance_angvel = 0.01):
        """
        Argument
        --------
        variance_angvel :  The variance of the angular velocity of the wheels
        """

        # Variance of input signal
        variance_omega = (WHEEL_RADIUS/WHEEL_DISTANCE)**2*variance_angvel
        variance_v = (WHEEL_RADIUS/2)**2*variance_angvel
        cov_omega_v = WHEEL_RADIUS**2/WHEEL_DISTANCE * variance_angvel
        self.covariance_u = np.array([[variance_omega, cov_omega_v],
                                      [cov_omega_v, variance_v]])
        # Covariance of odometry estimate
        self.covariance_state = np.diag([(np.pi/90.0)**2, (0.001)**2, (0.001)**2])

        # For convenience, the matrix that picks out the states to fill the 6x1 pose vector
        self.state2pose = np.zeros((6, 3))
        self.state2pose[0,1] = 1.0
        self.state2pose[1,2] = 1.0
        self.state2pose[5, 0] = 1.0

        self.twist_cov = np.zeros(36)
        self.twist_cov[0] = self.covariance_u[0,0] # cov(v,v)
        self.twist_cov[5] = self.covariance_u[0,1] # cov(v, w)
        self.twist_cov[30] = self.covariance_u[0,1] # cov(w, v)
        self.twist_cov[35] = self.covariance_u[1,1] # cov(w,w)

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
        v = 0
        w = 0
        
        while not rospy.is_shutdown():

            self.rate.sleep()
            
            self.model_pose.pose.position.x = self.model_state[1]
            self.model_pose.pose.position.y = self.model_state[2]
                
            self.model_pose.pose.orientation.w = np.cos(self.model_state[0]/2)
            self.model_pose.pose.orientation.z = np.sin(self.model_state[0]/2)

            # The covariance is row-major 6x6, that means
            #[cxx, cxy, cxz, cxr, cxp, cxya,
            # cxy, cyy, cyz, cyr, cyp, cyya,
            # czx, czy, czz, czr, czp, czya,
            # crx, cry, crz, crr, crp, crya,
            # cpx, cpy, cpz, cpr, cpp, cpya,
            # cyax, cyay, cyaz, cyar, cyap, cyaya]
            pose_cov = np.dot(self.state2pose,
                              np.dot(self.covariance_state, self.state2pose.T))
            
            self.model_pose.covariance[:] = np.ravel(pose_cov)

            
            self.model_twist.twist.linear.x = v
            self.model_twist.twist.angular.z = w
            self.model_twist.covariance[:] = self.twist_cov
            

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

            # Update the state
            if (self.wl is not None) and (self.wr is not None):
                self.current_time = rospy.get_time()
                dt  = self.current_time - self.last_time
                self.last_time = self.current_time
            
                w = (self.wr-self.wl) *f1
                v = (self.wr+self.wl)*f2
                delta_th = w*dt
                delta_s = v*dt
                th = self.model_state[0]
                th_mid = th + 0.5*delta_th
                cth = np.cos(th_mid)
                sth = np.sin(th_mid)
                
                self.model_state += np.array([delta_th,
                                              delta_s*cth,
                                              delta_s*sth])

                # Jacobians
                FzT = np.array([[1, -delta_s*sth, delta_s*cth],
                                [0,1,0],
                                [0,0,1]])
                FuT = np.array([[dt, -0.5*dt*delta_s*sth, 0.5*dt*delta_s*cth],
                                [0, dt*cth, dt*sth]])

                self.covariance_state = np.dot(FzT.T, np.dot(self.covariance_state, FzT)) + \
                    np.dot(FuT.T, np.dot(self.covariance_u,FuT))
                

if __name__ == '__main__':

    try:
        aux = MyOdometryPublisher()
        aux.main()

    except (rospy.ROSInterruptException, rospy.ROSException("topic was closed during publish()")):
        pass
