# ARIAC (Agile Robotics for Industrial Automation Competition) 2020

This project was developed as a course project for ENPM809B - Building a Manufacturing Robot Software Systems at the University of Maryland, College Park. For more details about the competition, please visit official competition page [here](https://www.nist.gov/el/intelligent-systems-division-73500/agile-robotics-industrial-automation-competition).<br>
Group members-
* [Saumil Shah](https://github.com/SaumilShah66)
* [Varun Asthana](https://github.com/varunasthana92)
* [Markose Jacob](https://github.com/markosej11)
* [Nalin Das](https://github.com/nalindas9)
* Aditya Goswami

Sequential development of the project was achieved in a total of 5 mile stones, with the last being the integration of all.

<p align="center">
<img src="https://github.com/varunasthana92/face_swap_classical_deep_learning/blob/master/Results/rambo_girl.gif">
</p>

## Dependencies
* Ubuntu 18.04
* ROS Melodic
* Gazebo >= 9.14
* GEAR Software (ARIAC 2020)
* MoveIt
* C++ 11/14

## How to build and run
To run the package rwa_final_4, do the following steps after creating an ROS ariac work spcae:
1) Clone the ROS package into the src directory of your ariac work space
2) Run
	`$ catkin build` 
3) Launch Gazebo along with moveit by using 
	`$ roslaunch rwa_final_4 rwa_final.launch load_moveit:=true`
4) To run the node, on another terminal execute
	`$ rosrun rwa_final_4 rwa_final_node`

## Test cases
Various test cses for different combination of order item and agility challenges has been provided in the directory as yaml files
`rwa_final_4)/test/`
<br>
To change the test file, in the launch directory edit the 
`rwa_final.launch` file at line 28 
'-f $(find rwa_final_4)/test/hpo_2.yaml'

__Note__<br>
* We are reading the order in reverse i.e, for order zero we first pick up part2, part1 and finally part0.

* Highest priority is always given for high priority order. 

* We have given preference for conveyor part so as soon as parts start to spawn on the conveyor we pick up the parts on the conveyor that is need to complete the order before we pick up static parts. 

* You may have to run the code 4 - 5 times to be able to successfully pick up the red pistion part as the thickness of the part is very less and the arm of the robot gets attached to the conveyor and the robot messes up.

## Known issues
* At times we observe some random behaviour in Gazebo which are not inline with the code logic. In such situation please rerun the entire code.