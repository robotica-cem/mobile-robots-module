#!/usr/bin/env python  
import sys
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RightHandRuleController:
    def __init__(self, v0=0.4 ):
        """
        Arguments
        --------------
        v0     :   Max linear velocity
        """
        self.scan_listener = rospy.Subscriber('/laser/scan', LaserScan,
                                              self.scan_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel' , Twist, queue_size=1 )
        self.rate = rospy.Rate(10.0)
        self.v0 = v0
        self.scan = None

    def scan_callback(msg):
        """Called when a new scan is available from the lidar. """
        self.scan = msg

        
    def follow_right_hand_wall(self):
        while not rospy.is_shutdown():
            if self.scan is not None:
                #--------------------------------------------------------------
                # Your code here
                # 1) Check distance to wall/obstacle in front and to the right.
                # 2) Based on the information in 1) calculate a suitable angular
                #    velocity, w.
                # 3) Publish the Twist message to steer the puzzlebot.
                
                v = self.v0
                w = 0.0
                
                msg = geometry_msgs.msg.Twist()
                msg.angular.z = w
                msg.linear.x = v 
                print(msg)
                self.vel_pub.publish(msg)
                
                #--------------------------------------------------------------
                #--------------------------------------------------------------
                
            self.rate.sleep()        
            


def get_distance_in_sector(scan, start_angle, end_angle) :
    """Returns the distance in m in the given sector by taking the average of the
    range scans.
    
    Arguments
    ---------
    scan : LaserScan
    start_angle : float
        Start angle of sector, in radians, where 0 is straight ahead.
    end_angle : float
        End angle of sector, in radians, where 0 is straight ahead.

    Tests
    -----
    >>> scan = generate_test_scan()
    >>> start_angle = -np.pi/2
    >>> end_angle = start_angle + np.pi/180 # One degree sector
    >>> expected_distance = np.mean([0,0.1,0.2,0.3])
    >>> np.abs(get_distance_in_sector(scan, start_angle, end_angle) - expected_distance) < 1e-8
    True
    >>> end_angle = np.pi/2
    >>> start_angle = end_angle - 2*np.pi/180 # Two degree sector
    >>> expected_distance = np.mean(np.arange(720-8, 720)/10)
    >>> np.abs(get_distance_in_sector(scan, start_angle, end_angle) - expected_distance) < 1e-8
    True
    """
    num_scans = len(scan.ranges)

    #--------------------------------------------------------------
    # Your code here. For documentation of the LaserScan message:
    # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
    #
    # 1) Find the indices into scan.ranges corresponding to the start_ange and end_angle
    # 2) Compute the average range in the sector using np.mean()
    #--------------------------------------------------------------
    return 0

def generate_test_scan():
    """Function used for testing. Will create and return a LaserScan message"""
    scan = LaserScan()
    scan.angle_min = -np.pi/2
    scan.angle_max = np.pi/2
    num_scans = 720 # 4 lines per degree
    scan.ranges = np.arange(0, 720) / 10
    scan.angle_increment = np.pi/num_scans
    scan.range_min = 0.1
    scan.range_max = 30

    return scan

if __name__ == '__main__':
    if len(sys.argv) > 1:
        if sys.argv[1] == "--test":
            import doctest
            doctest.testmod()
            sys.exit(0)

    rospy.init_node('Follow right hand wall')
    rhw = RightHandRuleController()
    rhw.follow_right_hand_wall()
    


