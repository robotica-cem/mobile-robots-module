#!/usr/bin/env python
""" Implements a "ground truth" odometry using the model state information from Gazebo.
"""
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance,TwistWithCovariance
from gazebo_msgs.msg import ModelState,ModelStates

class PosePublisher():
    def __init__(self):

        rospy.init_node('PosePublisher')

        # Subscribe to the model_states topic to obtain the position and heading of the puzzlebot
        rospy.Subscriber("gazebo/model_states", ModelStates, self.callback)
        # Publish the bot pose to a topic
        self.pub = rospy.Publisher("/true_odometry", Odometry)
        
        self.model_state = None
        self.initial_state = None
        self.bot_pose = PoseStamped()

        self.br = tf2_ros.TransformBroadcaster() # For broadcasting the transform from world to bot
        self.brStatic = tf2_ros.StaticTransformBroadcaster() # For broadcasting transform from world to initial pose of bot

    def main(self):

        # If there's an object attached to the ee we want it to follow its trajectory
        while not rospy.is_shutdown():
            if self.initial_state is None:
                if self.model_state is not None:
                    # Get the initial pose 
                    t = PoseStamped()
                    t.header.stamp = rospy.Time.now()
                    t.header.frame_id = "world"
                    t.child_frame_id = "bot_initial"
                    t.transform.translation = self.model_state.pose.position
                    t.transform.rotation = self.model_state.pose.orientation
                    self.initial_state = t
                    self.brStatic.sendTransform(t)
            if self.model_state is not None:
                # Publish the state of the puzzlebot
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "world"
                t.child_frame_id = "pbot"
                t.transform.translation = self.model_state.position
                t.transform.rotation = self.model_state.orientation
                self.br.sendTransform(t)


                    
            
            if self.model_state is not None and self.initial_state is not None:
                self.bot_pose.position.x = self.model_state.position.x - self.initial_state.position.x
                self.bot_pose.position.y = self.model_state.position.y - self.inital_state.position.y
                
            if self.attached[self.frame] and self.bool_changed:

                req = AttachRequest()
                req.model_name_1 = "xarm6"
                req.link_name_1 = "link6"
                req.model_name_2 = self.object
                req.link_name_2 = "link"

                self.attach_srv.call(req)

            elif self.bool_changed:

                req = AttachRequest()
                req.model_name_1 = "xarm6"
                req.link_name_1 = "link6"
                req.model_name_2 = self.object
                req.link_name_2 = "link"

                self.deattach_srv.call(req)


    def callback(self, data):

        # Map Gazebo and TF
        aux_idx = data.name.index(self.targets[self.task_id])
        self.target_pose.pose = data.pose[aux_idx]

        # We uptdate tf2 frames so that they have the same postiion as in Gazebo, if the object is attached we will keep
        # the same position relative to the ee.
        for target in self.targets:

            if (target != self.frame) or not (self.attached[target]):
                aux_idx = data.name.index(target)
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "sensor_frame"
                t.child_frame_id = target
                t.transform.translation = data.pose[aux_idx].position
                t.transform.rotation = data.pose[aux_idx].orientation
                self.br.sendTransform(t)

            else:
                # print("Attached")
                tf = self.tfBuffer.lookup_transform("sensor_frame", "link_attach", rospy.Time(), rospy.Duration(0.5))
                tf.child_frame_id = target
                tf.header.stamp = rospy.Time.now()
                self.br.sendTransform(tf)


    def sendGoal(self,req):

        # Proposed state machine to control the goals (currently 1)
        if req.action == "place":
            if self.status == 1:
                try:
                    aux_pose = self.targets_place[self.place_id]
                    self.place_id += 1
                    status = True
                    self.status = 0

                except:
                    aux_pose = "End"
                    status = True

            else:
                status = False
                aux_pose = "None, invalid place request! "

            print(aux_pose)
            return RequestGoalResponse(aux_pose,status)

        elif req.action == "pick":
            if self.status == 0:
                try:
                    aux_pose = self.targets_pick[self.task_id]
                    self.task_id += 1
                    status = True
                    self.status = 1

                except:
                    status = True
                    aux_pose = "End"

            else:
                status = False
                aux_pose = "None, invalid pick request! "

            print("target: " + aux_pose)
            return RequestGoalResponse(aux_pose,status)

if __name__ == '__main__':

    try:
        aux = StateGatherer()
        aux.main()

    except (rospy.ROSInterruptException, rospy.ROSException("topic was closed during publish()")):
        pass
