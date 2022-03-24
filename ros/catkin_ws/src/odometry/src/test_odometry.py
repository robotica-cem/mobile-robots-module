#!/usr/bin/env python
import numpy as np
import unittest
from reference_odometry import pose2np,transform_between_poses
from geometry_msgs.msg import Pose
from tf import transformations as trf

class TestOdometry(unittest.TestCase):

    def test_transform_translation(self):
        pb = Pose() # The pose of the base frame
        pb.position.x = 1.0

        pf = Pose() # The pose of the follower frame
        pf.position.x = -1.0

        t = transform_between_poses(pb, pf)

        print(t)
        texp = np.array([[1, 0, 0, -2.0],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
        self.assertTrue(np.allclose(pose2np(t), texp))

    def test_transform_rotation(self):
        th = np.pi/2
        pb = Pose() # The pose of the base frame
        pb.orientation.x = np.sin(th/2)
        pb.orientation.w = np.cos(th/2)

        pf = Pose() # The pose of the follower frame

        t = transform_between_poses(pb, pf)

        print(t)
        texp = np.array([[1, 0, 0, .0],
                         [0, 0, 1, 0],
                         [0, -1, 0, 0],
                         [0, 0, 0, 1]])
        self.assertTrue(np.allclose(pose2np(t), texp))


if __name__ == '__main__':
    unittest.main()

        
