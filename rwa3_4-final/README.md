# ENPM809B-Grp4
To run the package rwa3_4, simply do the following steps
1) Copy and paste the package rwa3_4 into the src directory of your ariac work space
2) Run 
	$ catkin build 
3) Launch Gazebo along with moveit by using 
	$ roslaunch rwa3_4 rwa3.launch load_moveit:=true
4) To run the node, on another terminal do
	$ rosrun rwa3_4 rwa3_node

note:
In rwa3-sample.yaml file, we have made disk_part_green_9 as faulty instead of disk_part_bule as the robot never has to pick up any faulty parts with the default sample.yaml file provided.


