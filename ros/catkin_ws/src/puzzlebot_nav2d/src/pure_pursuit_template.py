#!/usr/bin/env python  
import sys
import rospy
import numpy as np
from functools import partial
import tf2_ros
import geometry_msgs.msg
from tf2_geometry_msgs import PointStamped

def go_to_point_controller(x, y, vmax, Kth, alpha):
    """
    Calculates the desired linear- and angular velocities to move the robot to a point.
    Used for the final step in the pure pursuit to approach and stay at the goal position.

    Arguments
    ---------
    x     :  float
      The x-coordinate of the goal in the reference frame of the robot, in m
    y     :  float
      The x-coordinate of the goal in the reference frame of the robot, in m
    vmax  :  float
      The maximum linear velocity, in m/s
    Kth   :  float
      Gain for the direction controller
    alpha :  float
      Parameter for calculating the linear velocity as it approaches the goal

    Returns
    ---------
    w   :  float
      The angular velocity in rad/s
    v   :  float
      The linear velocity in m/s
    d   :  float
      The distance to the goal

    """

    v = 0.0
    w = 0.0
    d = 0.0
    return w, v, d
    
def steer_towards_point_controller(x, y, v):
    """
    Given an intermediate goal point and a (constant) linear velocity, calculates the
    angular velocity that will steer the robot towards the goal such that the robot
    moves in a circular arc that passes through the intermediate goal point.

    Arguments
    ---------
    x     :  float
      The x-coordinate of the goal in the reference frame of the robot, in m
    y     :  float
      The x-coordinate of the goal in the reference frame of the robot, in m
    v     :  float
      The linear velocity in m/s

    Returns
    ---------
    w   :  float
      The angular velocity in rad/s
    """

    w = 0.0
    return w

    
class PurePursuitController:
    def __init__(self, look_ahead_dist, max_vel,
                 go_to_point_control, steer_to_point_control ):
        """
        Arguments
        --------------
        look_ahead_distance     :  float
           As the name suggests.
        max_vel                 :  float
           The maximum linear velocity
        go_to_point_control     :  function object
           Function that computes the control for going to a point
        steer_to_point_control  :  function object
           Function that computes the control for steering to a point
        """
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.vel_pub = rospy.Publisher('/cmd_vel' , geometry_msgs.msg.Twist, queue_size=1 )
        self.rate = rospy.Rate(10.0)

        self.L = look_ahead_distance
        self.vmax = max_vel
        self.go2point = go_to_point_control
        self.steer2point = steer_to_point_control

        
    def follow_trajectory(self, waypoints, frame):
        """
        Follows the given trajectory, and stops at the goal, which is the final
        waypoint in the sequence of waypoints

        Arguments
        ---------
        waypoints : array-like
           List of waypoints [(x0, y0), (x1, y1), ..., (xN, yN)]
        frame     : string
           Name of the reference frame in which the waypoints are provided.
           There must exist a transfrom between this frame and the 'base_link'
           frame of the robot.
        """
        
        msg = geometry_msgs.msg.Twist()
        print(msg)
        
        current_wp_idx = 0
        next_wp_idx = 1
        current_wp = PointStamped()
        current_wp.header.frame_id = frame
        next_wp = PointStamped()
        next_wp.header.frame_id = frame

        goal = waypoints[-1]
        
        #if not rospy.is_shutdown():
        for cwp_, nwp_ in zip(waypoints(:-1), waypoints(1:)):
            current_wp.x = cwp_[0]
            current_wp.y = cwp_[1]
            next_wp.x = nwp_[0]
            next_wp.y = nwp_[1]

            # If next waypoint is goal, then just go there
            if nwp_ == goal:
                tol = 1e-2 # Stop when within 1cm of goal
                dist = 1
                while dist > tol:
                    # Transform the goal point
                    next_wp.header.stamp = rospy.Time(0)
                    next_wp_b = self.tf_buffer.transform(next_wp, 'base_link',
                                                    timeout = rospy.Duration(1))
                    w, v, dist = self.go2point(next_wp_b.x, next_wp_b.y, self.vmax)

                    msg.angular.z = w
                    msg.linear.x = v 

                    if not rospy.is_shutdown():
                        self.vel_pub.publish(msg)
                        self.rate.sleep()
                    else:
                        return
            else:
                # Find the intermediate goal point
                beta = 0.0
                while not np.isnan(beta) and beta < 1.0:
                    # Obtain the goal point expressed in the reference frame of base_link
                    current_wp.header.stamp = rospy.Time(0)
                    next_wp.header.stamp = rospy.Time(0)
                    current_wp_b = self.tf_buffer.transform(current_wp, 'base_link',
                                                    timeout = rospy.Duration(1))
                    next_wp_b = self.tf_buffer.transform(next_wp, 'base_link',
                                                    timeout = rospy.Duration(1))
                    pg, beta = self.get_goal_point([current_wp_b.x, current_wp_b.y],
                                                   [next_wp_b.x, next_wp_b.y],
                                                   self.L)
                    w =  self.steer2point(pg[0], pg[1], self.vmax)
                    msg.angular.z = w
                    msg.linear.x = self.vmax 
                    if not rospy.is_shutdown():
                        self.vel_pub.publish(msg)
                        self.rate.sleep()
                    else:
                        return
                    
                if np.isnan(beta):
                    # Must mean that trajectory is further than the look-ahead distance
                    
                    

if __name__ == '__main__':
    rospy.init_node('Pure_pursuit')
    L = rospy.get_param("~look_ahead_distance", 0.3)
    vmax = rospy.get_param("~vel_lin_max", 0.6)
    Kth = rospy.get_param("~K_theta", 4)
    alpha = rospy.get_param("~alpha", 4)
    frame = rospy.get_param("~frame")
    waypoints = np.asarray(rospy.get_param("~waypoints"), [-4,-4, -4, 2])
    waypoints = np.reshape(waypoints, (-1, 2))
    
    
    ppc = PurePursuitController(L, vmax,
                                partial(go_to_point_controller(Kth=Kth, alpha=alpha)),
                                steer_towards_point_controller)
    
    ppc.follow_trajectory(waypoints, frame)
    


