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
from panda_controller.srv import MoveToGoal, MoveToGoalResponse

class Funac_PicknPlace:
    def __init__(self):
        self.object_pos = None
        self.surface_gripper_pub = rospy.Publisher('/surface_gripper/joint_state', JointState, queue_size=10)

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
        table_pose_stamped.header.frame_id = "base_link"  # You can use "world" or another frame
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
        table_pose_stamped.header.frame_id = "base_link"  # You can use "world" or another frame
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
        table_pose_stamped.header.frame_id = "base_link"  # You can use "world" or another frame
        table_pose_stamped.pose.position.x = table_x
        table_pose_stamped.pose.position.y = table_y
        table_pose_stamped.pose.position.z = table_z
        table_pose_stamped.pose.orientation.w = 1.0  # No rotation

        # Add the table to the planning scene using PlanningSceneInterface
        planning_scene_interface.add_box(table_name, table_pose_stamped, (table_size_x, table_size_y, table_size_z))
        '''
        # Add a small object (box) to the scene
        obj_name = "object"
        # OBJ SIZE CAN DE DERIVED FROM DOPE FOR SIMPLICITY WE KNOW THE SIZE
        self.obj_size_x = 0.04
        self.obj_size_y = 0.04
        self.obj_size_z = 0.1

        # Create PoseStamped for the object
        object_pose_stamped = geometry_msgs.msg.PoseStamped()
        object_pose_stamped.header.frame_id = "base_link"  # Use "world" or another frame
        object_pose_stamped.pose.position.x = self.object_pos.x
        object_pose_stamped.pose.position.y = self.object_pos.y
        object_pose_stamped.pose.position.z = self.object_pos.z
        object_pose_stamped.pose.orientation.w = 1.0  # No rotation

        # Add the object to the planning scene using PlanningSceneInterface
        planning_scene_interface.add_box(obj_name, object_pose_stamped, (self.obj_size_x, self.obj_size_y, self.obj_size_z))
        '''
        rospy.loginfo(f"Known objects: {planning_scene_interface.get_known_object_names()}")
        
    def move_to_pick(self, move_group):

        # Create PoseStamped
        '''
        pick_pose = geometry_msgs.msg.PoseStamped()
        orientation = tf.quaternion_from_euler(np.pi/2, 0, -np.pi/2)
        pick_pose.header.frame_id = "Base"  # You can use "world" or another frame
        pick_pose.pose.position.x = 0.43
        pick_pose.pose.position.y = 0.0
        pick_pose.pose.position.z = 0.23

        rospy.loginfo(f"Move result: {orientation}")
        
        pick_pose.pose.orientation.x = 0.5
        pick_pose.pose.orientation.y = 0.5
        pick_pose.pose.orientation.z = -0.5
        pick_pose.pose.orientation.w = 0.5
        '''
        '''
        pick_pose = geometry_msgs.msg.PoseStamped()
        orientation = tf.quaternion_from_euler(np.pi/2, 0, -np.pi/2)
        pick_pose.header.frame_id = "Base"  # You can use "world" or another frame
        pick_pose.pose.position.x = 0.2979053574361228
        pick_pose.pose.position.y = -0.25666041951313195
        pick_pose.pose.position.z = 0.6597463465652358

        rospy.loginfo(f"Move result: {orientation}")
        
        pick_pose.pose.orientation.x = 0.6985950253283014
        pick_pose.pose.orientation.y = 0.027741303051240118
        pick_pose.pose.orientation.z = 0.004018162159235831
        pick_pose.pose.orientation.w = 0.7149680168122426
        '''
        pick_pose = geometry_msgs.msg.PoseStamped()
        #orientation = tf.quaternion_from_euler(0, -np.pi/2, 0)
        pick_pose.header.frame_id = "base_link"  # You can use "world" or another frame
        pick_pose.pose.position.x = 0.3
        pick_pose.pose.position.y = 0
        pick_pose.pose.position.z = 0.07
        
        pick_pose.pose.orientation.x = 1.0
        pick_pose.pose.orientation.y = 0.0
        pick_pose.pose.orientation.z = 0.0
        pick_pose.pose.orientation.w = 0.0

        move_group.set_pose_target(pick_pose)

        result=move_group.go(wait=True)
        rospy.loginfo(f"Move result: {result}")

    def move_to_place(self, move_group):

        # Create PoseStamped
        place_pose = geometry_msgs.msg.PoseStamped()
        #orientation = tf.quaternion_from_euler(0, np.pi/2, 0)
        place_pose.header.frame_id = "base_link"  # You can use "world" or another frame
        place_pose.pose.position.x = 0.0
        place_pose.pose.position.y = -0.4
        place_pose.pose.position.z = 0.3
        
        place_pose.pose.orientation.x = 1.0
        place_pose.pose.orientation.y = 0.0
        place_pose.pose.orientation.z = 0.0
        place_pose.pose.orientation.w = 0.0
        
        move_group.set_pose_target(place_pose)
        result=move_group.go(wait=True)

        rospy.loginfo(f"Move result: {result}")

    def turn_suction_cup_on(self):
        gripper_state = JointState()
        gripper_state.header.seq = 0
        gripper_state.header.stamp.secs = 0
        gripper_state.header.stamp.nsecs = 0
        gripper_state.header.frame_id = 'base_link'
        gripper_state.name = ['gripper']
        gripper_state.position = [1.0]
        gripper_state.velocity = [0.0]
        gripper_state.effort = [0.0]
        rospy.loginfo(gripper_state)
        self.surface_gripper_pub.publish(gripper_state)

    def turn_suction_cup_off(self):
        gripper_state = JointState()
        gripper_state.header.seq = 0
        gripper_state.header.stamp.secs = 0
        gripper_state.header.stamp.nsecs = 0
        gripper_state.header.frame_id = 'base_link'
        gripper_state.name = ['gripper']
        gripper_state.position = [0.0]
        gripper_state.velocity = [0.0]
        gripper_state.effort = [0.0]
        rospy.loginfo(gripper_state)
        self.surface_gripper_pub.publish(gripper_state)

    def move_joints(self, move_group):

        perch_pos = [0.00006621913274890453, -1.1895457475331512, -0.027683611198964383, -0.000043446390155418095, -0.314483173922748, 0.0]

        move_group.set_joint_value_target(perch_pos)
        result = move_group.go(wait=True)

        rospy.loginfo(f"Move to perch: {result}")

    def move_to_goal(self, req):
        self.object_pos = req.object_pos # Set the object's position
        self.goal_pos = req.goal_pos  # Set the goal position

        rospy.loginfo(f"Received request to move object from {self.object_pos} to {self.goal_pos}")
    
        moveit_commander.roscpp_initialize(sys.argv)
        
        planning_scene_interface = moveit_commander.PlanningSceneInterface()
        move_group = moveit_commander.MoveGroupCommander("manipulator")
        move_group.set_planning_time(45.0)
        rospy.sleep(1.0)

        self.add_collision_objects(planning_scene_interface)
        rospy.sleep(1.0)
        
        self.move_to_pick(move_group)
        rospy.sleep(1.0)

        self.turn_suction_cup_on()
        rospy.sleep(1.0)

        self.move_to_place(move_group)
        rospy.sleep(1.0)

        self.turn_suction_cup_off()
        rospy.sleep(1.0)

        current_pose = move_group.get_current_pose().pose
        rospy.loginfo(f"Current Pose: {current_pose}")

        return MoveToGoalResponse(success=True)
    

    def main(self):
        rospy.init_node("funac_pick_and_place")

        rospy.Service('move_to_goal', MoveToGoal, self.move_to_goal)

        rospy.spin()

if __name__ == "__main__":
    pick_and_place = Funac_PicknPlace()
    pick_and_place.main()
