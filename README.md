# ARIAC (Agile Robotics for Industrial Automation Competition) 2020

This was developed as a course project for ENPM809B - Building a Manufacturing Robot Software Systems. For more details about the competition, please visit official competition page [here](https://www.nist.gov/el/intelligent-systems-division-73500/agile-robotics-industrial-automation-competition). Group members are,
* [Saumil Shah](https://github.com/SaumilShah66)
* [Varun Asthana](https://github.com/varunasthana92)
* [Markose Jacob](https://github.com/markosej11)
* [Nalin Das](https://github.com/nalindas9)
* Aditya Goswami

To run the package rwa5_4, simply do the following steps
1) Copy and paste the package rwa5_4 into the src directory of your ariac work space
2) Run 
	`$ catkin build` 
3) Launch Gazebo along with moveit by using 
	`$ roslaunch rwa5_4 rwa5.launch load_moveit:=true`
4) To run the node, on another terminal do
	`$ rosrun rwa5_4 rwa5_node`

note:
* We are reading the order in reverse i.e, for order zero we first pick up part2, part1 and finally part0. We are doing this for both the shipment.

* Highest priority is always given for high priority order. 

* We have given preference for conveyor part so as soon as parts start to spawn on the conveyor we pick up the parts on the conveyor that is need to complete the order before we pick up static parts. 

* You may have to run the code 4 - 5 times to be able to successfully pick up the red pistion part as the thickness of the part is very less and the arm of the robot gets attached to the conveyor and the robot messes up.

* We have not made any changes to the .yaml file

* We were not able to pick the piston part from agv tray. Robot arm was not able to reach to that height and hence bypassing quality and pose check for piston parts.



