import rospy
import sys
import moveit_commander
from sensor_msgs.msg import JointState
from moveit_commander import MoveGroupCommander
from moveit_commander import PlanningSceneInterface
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import shape_msgs.msg
import tf.transformations as tf
import numpy as np
from robot_controller.srv import MoveToGoal, MoveToGoalResponse

class Funac_PicknPlace:
    def __init__(self):
        self.object_pos = None

    def add_collision_objects(self, planning_scene_interface):
        # Add a table (box) to the scene
        table_name = "robot_table"
        table_size_x = 0.98
        table_size_y = 0.78
        table_size_z = 0.78
        table_x = 0.0
        table_y = 0.0
        table_z = -0.39

        # Create PoseStamped for the table object
        table_pose_stamped = geometry_msgs.msg.PoseStamped()
        table_pose_stamped.header.frame_id = "world"  # You can use "world" or another frame
        table_pose_stamped.pose.position.x = table_x
        table_pose_stamped.pose.position.y = table_y
        table_pose_stamped.pose.position.z = table_z
        table_pose_stamped.pose.orientation.w = 1.0  # No rotation

        # Add the table to the planning scene using PlanningSceneInterface
        planning_scene_interface.add_box(table_name, table_pose_stamped, (table_size_x, table_size_y, table_size_z))

        table_name = "faulty_conveyor"
        table_size_x = 0.3
        table_size_y = 0.78
        table_size_z = 0.78
        table_x = 0.65
        table_y = 0.0
        table_z = -0.39

        # Create PoseStamped for the table object
        table_pose_stamped = geometry_msgs.msg.PoseStamped()
        table_pose_stamped.header.frame_id = "world"  # You can use "world" or another frame
        table_pose_stamped.pose.position.x = table_x
        table_pose_stamped.pose.position.y = table_y
        table_pose_stamped.pose.position.z = table_z
        table_pose_stamped.pose.orientation.w = 1.0  # No rotation

        # Add the table to the planning scene using PlanningSceneInterface
        planning_scene_interface.add_box(table_name, table_pose_stamped, (table_size_x, table_size_y, table_size_z))

        table_name = "next_conveyor"
        table_size_x = 0.78
        table_size_y = 0.3
        table_size_z = 0.78
        table_x = 0.0
        table_y = 0.55
        table_z = -0.39

        # Create PoseStamped for the table object
        table_pose_stamped = geometry_msgs.msg.PoseStamped()
        table_pose_stamped.header.frame_id = "world"  # You can use "world" or another frame
        table_pose_stamped.pose.position.x = table_x
        table_pose_stamped.pose.position.y = table_y
        table_pose_stamped.pose.position.z = table_z
        table_pose_stamped.pose.orientation.w = 1.0  # No rotation

        # Add the table to the planning scene using PlanningSceneInterface
        planning_scene_interface.add_box(table_name, table_pose_stamped, (table_size_x, table_size_y, table_size_z))

        # Add a small object (box) to the scene
        obj_name = "object"
        # OBJ SIZE CAN DE DERIVED FROM DOPE FOR SIMPLICITY WE KNOW THE SIZE
        self.obj_size_x = 0.04
        self.obj_size_y = 0.04
        self.obj_size_z = 0.1

        # Create PoseStamped for the object
        object_pose_stamped = geometry_msgs.msg.PoseStamped()
        object_pose_stamped.header.frame_id = "world"  # Use "world" or another frame
        object_pose_stamped.pose.position.x = self.object_pos.x
        object_pose_stamped.pose.position.y = self.object_pos.y
        object_pose_stamped.pose.position.z = self.object_pos.z
        object_pose_stamped.pose.orientation.w = 1.0  # No rotation

        # Add the object to the planning scene using PlanningSceneInterface
        planning_scene_interface.add_box(obj_name, object_pose_stamped, (self.obj_size_x, self.obj_size_y, self.obj_size_z))

        rospy.loginfo(f"Known objects: {planning_scene_interface.get_known_object_names()}")
        
    def move_to_pick(self, move_group):

        # Create PoseStamped
        pick_pose = geometry_msgs.msg.PoseStamped()
        orientation = tf.quaternion_from_euler(-np.pi, 0, np.pi*2)
        pick_pose.header.frame_id = "world"  # You can use "world" or another frame
        pick_pose.pose.position.x = 0.4
        pick_pose.pose.position.y = 0
        pick_pose.pose.position.z = 0.2534
        
        pick_pose.pose.orientation.x = orientation[0]
        pick_pose.pose.orientation.y = orientation[1]
        pick_pose.pose.orientation.z = orientation[2]
        pick_pose.pose.orientation.w = orientation[3]

        move_group.set_pose_target(pick_pose)
        result=move_group.go(wait=True)

        rospy.loginfo(f"Move result: {result}")

    def move_to_place(self, move_group):

        # Create PoseStamped
        place_pose = geometry_msgs.msg.PoseStamped()
        orientation = tf.quaternion_from_euler(-np.pi, 0, np.pi*2)
        place_pose.header.frame_id = "world"  # You can use "world" or another frame
        place_pose.pose.position.x = 0.0
        place_pose.pose.position.y = 0.4
        place_pose.pose.position.z = 0.2534
        
        place_pose.pose.orientation.x = orientation[0]
        place_pose.pose.orientation.y = orientation[1]
        place_pose.pose.orientation.z = orientation[2]
        place_pose.pose.orientation.w = orientation[3]

        move_group.set_pose_target(place_pose)
        result=move_group.go(wait=True)

        rospy.loginfo(f"Move result: {result}")

    def move_to_goal(self, req):
        self.object_pos = req.object_pos # Set the object's position
        self.goal_pos = req.goal_pos  # Set the goal position

        rospy.loginfo(f"Received request to move object from {self.object_pos} to {self.goal_pos}")
    
        moveit_commander.roscpp_initialize(sys.argv)
        
        planning_scene_interface = moveit_commander.PlanningSceneInterface()
        move_group = moveit_commander.MoveGroupCommander("funac_arm")
        move_group.set_planning_time(45.0)
        rospy.sleep(1.0)

        self.add_collision_objects(planning_scene_interface)
        rospy.sleep(1.0)

        self.move_to_pick(move_group)
        rospy.sleep(1.0)
        
        self.move_to_place(move_group)
        rospy.sleep(1.0)

        return MoveToGoalResponse(success=True)

    def main(self):
        rospy.init_node("funac_pick_and_place")

        rospy.Service('move_to_goal', MoveToGoal, self.move_to_goal)

        rospy.spin()

if __name__ == "__main__":
    pick_and_place = Funac_PicknPlace()
    pick_and_place.main()
