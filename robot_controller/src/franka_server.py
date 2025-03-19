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

class Franka_PicknPlace:
    def __init__(self):
        self.object_pos = None

    def open_gripper(self, posture):
        posture.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = [0.04, 0.04]
        point.time_from_start = rospy.Duration(0.5)
        posture.points.append(point)

    def closed_gripper(self, posture):
        posture.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = [0.0, 0.0]
        point.time_from_start = rospy.Duration(0.5)
        posture.points.append(point)

    def add_collision_objects(self, planning_scene_interface):
        # Add a table (box) to the scene
        table_name = "robot_table"
        table_size_x = 0.78
        table_size_y = 0.78
        table_size_z = 0.78
        table_x = 0.0
        table_y = 0.0
        table_z = -0.39

        # Create PoseStamped for the table object
        table_pose_stamped = geometry_msgs.msg.PoseStamped()
        table_pose_stamped.header.frame_id = "panda_link0"  # You can use "world" or another frame
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
        table_x = 0.55
        table_y = 0.0
        table_z = -0.39

        # Create PoseStamped for the table object
        table_pose_stamped = geometry_msgs.msg.PoseStamped()
        table_pose_stamped.header.frame_id = "panda_link0"  # You can use "world" or another frame
        table_pose_stamped.pose.position.x = table_x
        table_pose_stamped.pose.position.y = table_y
        table_pose_stamped.pose.position.z = table_z
        table_pose_stamped.pose.orientation.w = 1.0  # No rotation

        # Add the table to the planning scene using PlanningSceneInterface
        planning_scene_interface.add_box(table_name, table_pose_stamped, (table_size_x, table_size_y, table_size_z))

        table_name = "feeding_conveyor"
        table_size_x = 0.78
        table_size_y = 0.3
        table_size_z = 0.78
        table_x = 0
        table_y = -0.55
        table_z = -0.39

        # Create PoseStamped for the table object
        table_pose_stamped = geometry_msgs.msg.PoseStamped()
        table_pose_stamped.header.frame_id = "panda_link0"  # You can use "world" or another frame
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
        table_pose_stamped.header.frame_id = "panda_link0"  # You can use "world" or another frame
        table_pose_stamped.pose.position.x = table_x
        table_pose_stamped.pose.position.y = table_y
        table_pose_stamped.pose.position.z = table_z
        table_pose_stamped.pose.orientation.w = 1.0  # No rotation

        # Add the table to the planning scene using PlanningSceneInterface
        planning_scene_interface.add_box(table_name, table_pose_stamped, (table_size_x, table_size_y, table_size_z))

        # Add a small object (box) to the scene
        obj_name = "object"
        # OBJ SIZE CAN DE DERIVED FROM DOPE FOR SIMPLICITY WE KNOW THE SIZE
        self.obj_size_x = 0.068
        self.obj_size_y = 0.068
        self.obj_size_z = 0.1

        # Create PoseStamped for the object
        object_pose_stamped = geometry_msgs.msg.PoseStamped()
        object_pose_stamped.header.frame_id = "panda_link0"  # Use "world" or another frame
        object_pose_stamped.pose.position.x = self.object_pos.x
        object_pose_stamped.pose.position.y = self.object_pos.y
        object_pose_stamped.pose.position.z = self.object_pos.z
        object_pose_stamped.pose.orientation.w = 1.0  # No rotation

        # Add the object to the planning scene using PlanningSceneInterface
        planning_scene_interface.add_box(obj_name, object_pose_stamped, (self.obj_size_x, self.obj_size_y, self.obj_size_z))

        rospy.loginfo(f"Known objects: {planning_scene_interface.get_known_object_names()}")

    def pick(self, move_group):

        grasps = [moveit_msgs.msg.Grasp()]
        grasps[0].grasp_pose.header.frame_id = "panda_link0"
        
        #orientation = tf.quaternion_from_euler(np.pi/2, -np.pi/2, 0)
        orientation = tf.quaternion_from_euler(-np.pi, 0, np.pi*2)
        grasps[0].grasp_pose.pose.orientation.x = orientation[0]
        grasps[0].grasp_pose.pose.orientation.y = orientation[1]
        grasps[0].grasp_pose.pose.orientation.z = orientation[2]
        grasps[0].grasp_pose.pose.orientation.w = orientation[3]

        grasps[0].grasp_pose.pose.position.x = self.object_pos.x
        grasps[0].grasp_pose.pose.position.y = self.object_pos.y
        grasps[0].grasp_pose.pose.position.z = self.object_pos.z * 2 + 0.1034 - 0.02 
        #grasps[0].grasp_pose.pose.position.z = object_height + ee_offset - grabbing_space = 0.1834
        
        grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0"
        grasps[0].pre_grasp_approach.direction.vector.z = -1.0  # Move downwards
        grasps[0].pre_grasp_approach.min_distance = 0.02
        grasps[0].pre_grasp_approach.desired_distance = 0.04
        
        grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0"
        grasps[0].post_grasp_retreat.direction.vector.z = 1.0
        grasps[0].post_grasp_retreat.min_distance = 0.05
        grasps[0].post_grasp_retreat.desired_distance = 0.10

        self.open_gripper(grasps[0].pre_grasp_posture)
        self.closed_gripper(grasps[0].grasp_posture)
    
        move_group.set_support_surface_name("robot_table")
        result = move_group.pick("object", grasps)
        rospy.loginfo(f"Pick result: {result}")

        if result:  # Check if the pick was successful
            rospy.loginfo("Pick action successful")
            return True
        else:
            rospy.logwarn("Pick action failed")
            return False

    def place(self, move_group):
        """Perform the place operation."""
        place_location = [moveit_commander.PlaceLocation()]

        # Setting place location pose
        place_location[0].place_pose.header.frame_id = "panda_link0"
        
        # Orientation (Equivalent to tau/4 in C++)
        quaternion = tf.quaternion_from_euler(0, 0, 3.14159 / 2)
        place_location[0].place_pose.pose.orientation.x = quaternion[0]
        place_location[0].place_pose.pose.orientation.y = quaternion[1]
        place_location[0].place_pose.pose.orientation.z = quaternion[2]
        place_location[0].place_pose.pose.orientation.w = quaternion[3]

        # Position
        place_location[0].place_pose.pose.position.x = self.goal_pos.x
        place_location[0].place_pose.pose.position.y = self.goal_pos.y
        place_location[0].place_pose.pose.position.z = self.goal_pos.z

        # Pre-place approach
        place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0"
        place_location[0].pre_place_approach.direction.vector.z = -1.0
        place_location[0].pre_place_approach.min_distance = 0.03
        place_location[0].pre_place_approach.desired_distance =  0.06
        # Post-place retreat
        place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0"
        place_location[0].post_place_retreat.direction.vector.z = 1.0
        place_location[0].post_place_retreat.min_distance = 0.1
        place_location[0].post_place_retreat.desired_distance = 0.25

        # Open gripper after placing
        self.open_gripper(place_location[0].post_place_posture)

        # Set support surface
        move_group.set_support_surface_name("next_conveyor")

        # Execute place operation
        result = move_group.place("object", place_location)
        rospy.loginfo(f"Place result: {result}")

        if result:  # Check if the pick was successful
            rospy.loginfo("Place action successful")
            return True
        else:
            rospy.logwarn("Place action failed")
            return False
        
    def move(self, move_group):

        perch_pos = [0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966, 0.7853981633974483]

        move_group.set_joint_value_target(perch_pos)
        result = move_group.go(wait=True)

        rospy.loginfo(f"Move to perch: {result}")

    def move_to_goal(self, req):
        self.object_pos = req.object_pos # Set the object's position
        self.goal_pos = req.goal_pos  # Set the goal position

        rospy.loginfo(f"Received request to move object from {self.object_pos} to {self.goal_pos}")
    
        moveit_commander.roscpp_initialize(sys.argv)
        
        planning_scene_interface = moveit_commander.PlanningSceneInterface()
        move_group = moveit_commander.MoveGroupCommander("panda_arm")
        move_group.set_planning_time(45.0)
        rospy.sleep(1.0)

        # Publish joint states of perch position
        perch_pos = [0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966, 0.7853981633974483]
        if perch_pos != move_group.get_current_joint_values():
            rospy.loginfo(f"Moving to perch position...")
            self.move(move_group)
        
        rospy.loginfo(f"Robot in perch position!")

        self.add_collision_objects(planning_scene_interface)
        rospy.sleep(1.0)

        pick_result = self.pick(move_group)

        if not pick_result:
            rospy.logwarn("Pick failed, skipping place action")
            moveit_commander.roscpp_shutdown()
            return MoveToGoalResponse(success=False)
        
        rospy.sleep(1.0)
        
        place_result = self.place(move_group)

        if not place_result:
            rospy.logwarn("Place failed")
            moveit_commander.roscpp_shutdown()
            return MoveToGoalResponse(success=False)
        
        rospy.sleep(1.0)

        return MoveToGoalResponse(success=True)

    def main(self):
        rospy.init_node("panda_pick_and_place")

        rospy.Service('move_to_goal', MoveToGoal, self.move_to_goal)

        rospy.spin()

if __name__ == "__main__":
    pick_and_place = Franka_PicknPlace()
    pick_and_place.main()
