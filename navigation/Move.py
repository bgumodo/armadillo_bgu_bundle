#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String


class Move:

    def __init__(self):
        rospy.init_node('move_to_goal', anonymous=True)

        self.move_cmd = rospy.Subscriber("/navigation/move_cmd", String, self.move_call_back)
        self.pos_cmd = rospy.Subscriber("/navigation/pos_cmd", Pose, self.move_to_location)

        self.move_res = rospy.Publisher("/navigation/move_res", String, queue_size=5)

        self.locations = dict()

        self.locations["room_2"] = Pose(Point(-0.597, -0.423, 0), Quaternion(0, 0, 0.930, 0.368)) #initial room of the robot
        self.locations["door_zone_1"] = Pose(Point(-9.4, 4, 0), Quaternion(0, 0, 1, -0.023))
        self.locations["door_zone_2"] = Pose(Point(-9.105, 6.080, 0), Quaternion(0, 0, -0.373, 0.928))
        self.locations["room_4"] = Pose(Point(-4.04, 8.210, 0), Quaternion(0, 0, 0.771, 0.636)) #initial room of the can

    def move_call_back(self, data):
        loc = str(data.data)

        print "going to " + loc

        self.move_to_location(self.locations[loc])

    def move_to_location(self, location):
        move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")
        move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")

        goal = MoveBaseGoal()

        goal.target_pose.pose = location
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        move_base.send_goal(goal)

        move_base.wait_for_result(rospy.Duration(300))

        state = move_base.get_state()

        if state == GoalStatus.SUCCEEDED:
            self.move_res.publish("Success")
        else:
            self.move_res.publish("Falied")


if __name__ == '__main__':

    Move()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print 'Shutting down'
