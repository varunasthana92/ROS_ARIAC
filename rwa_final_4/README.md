# ENPM809B-Grp4
To run the package rwa_final_4, simply do the following steps:
1) Copy and paste the package rwa5_4 into the src directory of your ariac work space
2) Run 
	$ catkin build 
3) Launch Gazebo along with moveit by using 
	$ roslaunch rwa_final_4 rwa_final.launch load_moveit:=true
4) To run the node, on another terminal do
	$ rosrun rwa_final_4 rwa_final_node

note:
* We are reading the order in reverse i.e, for order zero we first pick up part (n), then part (n-1) ...  and finally part0. We are doing this for all the orders.

* Highest priority is always given for high priority order. 

* We have given preference for conveyor part so as soon as parts start to spawn on the conveyor we pick up the parts on the conveyor that is need to complete the order before we pick up static parts.


Doxygen Documentaion: 
* Documentation can be found in ~..../rwa_final_4/docs/html/index.html

If you want to generate another doxygen documentation then do the following:
* Delete 'docs' directory from the package

* Doxyfile config file can found inside the package. file name - "Doxyfile"

* Change directory to the directory with contains the Doxyfile. Below command is what we used on our system
	$ cd ariac_ws/src/ROS_ARIAC/rwa_final_4/

* Generate documentation 
	$ doxygen Doxyfile

* The above command will create a directory called 'docs' which has the documentation






