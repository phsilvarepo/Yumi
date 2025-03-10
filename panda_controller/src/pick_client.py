#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from panda_controller.srv import MoveToGoal

def move_to_goal_client(current_pos, goal_pos):
    rospy.wait_for_service('move_to_goal')
    try:
        move_to_goal = rospy.ServiceProxy('move_to_goal', MoveToGoal)
        response = move_to_goal(current_pos, goal_pos)
        if response.success:
            rospy.loginfo("Movement successful!")
        else:
            rospy.loginfo("Movement failed!")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node('move_to_goal_client')
    obj_position = Point(0.4, 0.0, 0.05)
    goal_position = Point(0.5, 0.0, 0.2)
    move_to_goal_client(obj_position, goal_position)

#IMPLEMENT YOLO CALLBACK TO SEND TO CONVEYORS
#IMPLEMENT POSE CALLBACK FROM DOPE