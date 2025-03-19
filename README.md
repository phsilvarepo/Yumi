# Yumi
ROS Noetic Package to control multiple robots through MoveIt package. The current setup utilizes Isaac Sim 4.5 to provide a simulation environment. The operations are activated through a service, this service requires the object position and the goal position and responds with a boolean to determine the success of the operation. 

## Package description:

isaac_sim_envs: USD files with custom environments for pick and place tasks, Yumi and Franka use gripper and Fanuc utilizes vaccum gripper

robot_controller: Service implementation

yumi_description, yumi_moveit_config : Package to enable MoveIt for ABB Yumi

fanuc_lrmate200id_moveit_plugins, fanuc_lrmate200id_moveit_config, fanuc_lrmate200id_support, fanuc_resources : Package to enable MoveIt for Funac LRMate 200ID

panda_moveit_config : Package to enable MoveIt for Franka Emika Panda

Note: yumi_description package contains Yumi's URDF and USD files. fanuc_lrmate200id_support contains Fanuc's URDF and USD files. Panda USD is available as franka_instanceable.usd on Isaac Sim Assets 

## How to use:
Launch demo launch file: 
roslaunch panda_moveit_config demo.launch

Run server node:
rosrun robot_controller franka_server.py

Run client node:
rosrun robot_controller pick_client.py
