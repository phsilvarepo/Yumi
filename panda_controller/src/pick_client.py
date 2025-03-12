#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PoseStamped
from panda_controller.srv import MoveToGoal
from std_msgs.msg import String

obj_position = None
goal_position = None

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

def yolo_callback(msg):
    global goal_position
    
    detected_objects = msg.data.strip("[]").replace("'", "").split(", ")

    if len(detected_objects) != 1:
        rospy.loginfo("YOLO is detecting several objects, can't associate labels!")
        return
    
    detected_class = detected_objects[0]
    rospy.loginfo(f"Detected class: {detected_class}")

    faulty_conveyor_pos = Point(0.55, 0.0, 0.12)
    next_conveyor_pos = Point(0.0, 0.55, 0.12)

    if detected_class == "soup":
        goal_position = next_conveyor_pos
    else: 
        goal_position = faulty_conveyor_pos

#Have to deal with multiple DOPE readings by subscribing to /dope/detected_objects     # vision_msgs/Detection3DArray of all detected objects
#instead of PoseStamped that contains information about several objects in a unorganized manner
def dope_callback(msg):
    global obj_position
    obj_position = msg.pose.position 

if __name__ == "__main__":
    rospy.init_node('move_to_goal_client')

    #rospy.Subscriber("/ultralytics/detection/classes", String, yolo_callback)
    #rospy.Subscriber("/dope/relative_soup_pose", PoseStamped, dope_callback)

    #obj_position = Point(0.0, -0.55, 0.05) Object on the right of robot in Franka env
    #goal_position = Point(0.0, 0.55, 0.12) Next Conveyor Franka
    #goal_position = Point(0.55, 0.0, 0.12) Faulty Conveyor Franka

    obj_position = Point(0.4, 0.0, 0.05) # Object in front of robot in Yumi env
    goal_position = Point(0.65, 0.0, 0.12) # Next Conveyor Yumi
    #goal_position = Point(0.55, 0.55, 0.12) Faulty Conveyor Yumi
    
    rate = rospy.Rate(10)  # 10 Hz loop rate
    while not rospy.is_shutdown():
        if obj_position and goal_position:
            rospy.loginfo(f"Object position: {obj_position}")
            rospy.loginfo(f"Goal position: {goal_position}")
            rospy.loginfo("Calling service...")

            move_to_goal_client(obj_position, goal_position)
            break  # Exit after one successful movement
        rate.sleep()

