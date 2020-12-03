# ENPM809B-Grp4
To run the package rwa_final_4, simply do the following steps:
1) Copy and paste the package rwa_final_4 into the src directory of your ariac work space
2) Run 
	$ catkin build 
3) Launch Gazebo along with moveit by using 
	$ roslaunch rwa_final_4 rwa_final.launch load_moveit:=true
4) To run the node, on another terminal do
	$ rosrun rwa_final_4 rwa_final_node

note:
* At times we observe some random behaviour in Gazebo which are not inline with the code logic. In such situation please rerun the entire code.

* We have given preference for conveyor part so as soon as parts start to spawn on the conveyor we pick up the parts on the conveyor that is need to complete the order before we pick up static parts.


Doxygen Documentaion: 
* Documentation can be found in ~..../rwa_final_4/docs/html/index.html

If you want to generate another doxygen documentation then do the following:
* Delete 'docs' directory from the package

* Doxygen config file can found inside the package. file name - "Doxyfile"

* Change directory to the directory that contains the Doxyfile. 

* Generate documentation 
	$ doxygen Doxyfile

* The above command will create a directory named 'docs' which has the documentation






