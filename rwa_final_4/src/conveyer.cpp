#include <algorithm>
#include <vector>
#include <string>
#include <unordered_map>
#include "competition.h"
#include <ros/ros.h>
#include "conveyer.h"

void Detection::emptyDetection() {
	part_set = false;
	picked_up = false;
	first_look_time = -1;
	second_look_time = -1;
	speed = -1;
	type = "";
}

static double giveCurrentTime() {
	return ros::Time::now().toSec();
}

double round (double x) {
    return std::floor(x + 0.5f);
}

void ConveyerParts::conveyerLogicalCameraCallback(const nist_gear::LogicalCameraImage& msg) {
	try {
		updateCurrentPoses();
		checkBoundaries();
		checkCurrentPartSet();
		// No part
		if(msg.models.size()==0 && current_detection.type.size()==0) {
			// ROS_DEBUG_STREAM_THROTTLE(2,"No parts detected on conveyer belt camera");
			return;
		}
		
		// When seen for the first time set the name, first seen time and first seen position
		if(msg.models.size()==1 && 	current_detection.type.size()==0) {
			auto current_pose = getPose_W(msg.models[0].pose);
			if(current_pose.position.y < part_read_limit) {
				// ROS_DEBUG_STREAM_THROTTLE(2, "Already settled part ");
				return;
			}else if(current_pose.position.z >= 0.9){
				return;
			}
			current_detection.type = msg.models[0].type;  // Setting name of the part if not assigned
			// ROS_DEBUG_STREAM("Product name on conveyer set to " << current_detection.type);
		}
		// First pose and time detection
		if(msg.models.size()==1 && current_detection.first_look_time == -1) {
			
			current_detection.first_look_time = giveCurrentTime();
			// ROS_DEBUG_STREAM("Conveyer : watched first");
			current_detection.first_pose = getPose_W(msg.models[0].pose);
			return;
		}
		// Second pose and time and speed
		if(msg.models.size()==1 && current_detection.second_look_time == -1) {
			double present_time = giveCurrentTime();
			// Too soon and giveClosestPart
			if(present_time - current_detection.first_look_time <= 2) {
				return;
			}
			// ROS_DEBUG_STREAM("Conveyer : watched second");
			current_detection.second_look_time = giveCurrentTime();
			// ROS_DEBUG_STREAM("Product on conveyer was watched second time at " << current_detection.second_look_time << " seconds");
			current_detection.second_pose = getPose_W(msg.models[0].pose);
			current_detection.current_pose = getPose_W(msg.models[0].pose);
			calculateSpeed();
			current_detection.part_set = true;
			allConveyerParts.push_back(current_detection);
			ROS_WARN_STREAM("------------Pushed------------ " << allConveyerParts.size());
			// current_detection.emptyDetection();
			return;
		}
	}
	catch(int n){
		ROS_WARN_STREAM("Problem error : " << n);
	}
}

void ConveyerParts::checkCurrentPartSet() {
	if (current_detection.part_set && current_detection.current_pose.position.y < part_read_limit) {
	// if(current_detection.part_set &&  giveCurrentTime() - current_detection.first_look_time > 3.0) {
		ROS_WARN_STREAM(" ------ empty current_detection " << current_detection.current_pose.position.y);
		current_detection.emptyDetection();
	}
}

bool ConveyerParts::checkPart(const std::string &part_name) {
	for(const auto &detection: allConveyerParts) {
		if(detection.type.compare(part_name) == 0) {
			return true;
		}
	}
	return false;
}

bool ConveyerParts::checkForPick() {
	
	checkBoundaries();
	checkCurrentPartSet();
	updateCurrentPoses();
	if(ready_for_pick) {
		double time_elapsed = giveCurrentTime() - pick_part.first_look_time;
		pick_part.current_pose.position.y = pick_part.first_pose.position.y - pick_part.speed*time_elapsed  ;
		double distance_left = pick_part.current_pose.position.y - pick_pose.position.y;
		// ROS_DEBUG_STREAM_THROTTLE(1,"Current distance from pickup location --> " << distance_left );
		if(distance_left <= 0.08) {
			ROS_WARN_STREAM("****** Try to pick up now ******");
			ready_for_pick = false;
			return true;
		}
	}
	return false;
}

bool ConveyerParts::giveClosestPart(const std::string &part_name, geometry_msgs::Pose &poseOnConveyer) {
	// ROS_WARN_STREAM_THROTTLE(1,"--- giveClosestPart ----" << allConveyerParts.size());
	checkBoundaries();
	for(int i = 0; i < allConveyerParts.size(); i++) {
		updateCurrentPoses();
		// checkCurrentPartSet();
		// ROS_WARN_STREAM_THROTTLE(1,"--- checking giveClosestPart----" << allConveyerParts[i].type << " of total " << allConveyerParts.size());
		if(allConveyerParts[i].type.compare(part_name) == 0 &&  !allConveyerParts[i].picked_up) {
			// ROS_WARN_STREAM("Found " << current_detection.type << " the part on conveyer");
			poseOnConveyer = allConveyerParts[i].current_pose;
			poseOnConveyer.position.y -= offset;
			estimated_time = giveCurrentTime() + offset/allConveyerParts[i].speed;
			// ROS_INFO_STREAM("Time was measured --> " << giveCurrentTime() << " estimated -->" << estimated_time);
			// ROS_INFO_STREAM("This pose would be great to pick up a part from conveyer "
			// 				<< " X : " << poseOnConveyer.position.x
			// 				<< " Y : " << poseOnConveyer.position.y
			// 				<< " Z : " << poseOnConveyer.position.z);
			pick_pose = poseOnConveyer;
			ready_for_pick = true;
			pick_part = allConveyerParts[i];
			allConveyerParts[i].picked_up = true;
			ROS_WARN_STREAM("--- found on conveyor ----");
			// allConveyerParts.erase(allConveyerParts.begin()+i);
			return true;
		}
	}
	// ROS_WARN_STREAM_THROTTLE(1,"--- giveClosestPart end ----" << allConveyerParts.size());
	return false;
}

void ConveyerParts::updateCurrentPoses() {
	for(auto &part: allConveyerParts) {
		double time_elapsed = giveCurrentTime() - part.first_look_time;
		part.current_pose.position.y = part.first_pose.position.y - part.speed*time_elapsed;
		// ROS_DEBUG_STREAM_THROTTLE(1,"New position of " << part.type << " is " 
		// 							<< " X : " << part.current_pose.position.x
		// 							<< " Y : " << part.current_pose.position.y
		// 							<< " Z : " << part.current_pose.position.z);
		// ROS_DEBUG_STREAM_THROTTLE(1,"Should reach 0 at : " << part.current_pose.position.y/part.speed);
	}
	if(current_detection.part_set) {
		// ROS_WARN_STREAM("--- update currrent ----" << allConveyerParts.size());
		// current_detection = allConveyerParts.back();
		double time_elapsed = giveCurrentTime() - current_detection.first_look_time;
		current_detection.current_pose.position.y = current_detection.first_pose.position.y - current_detection.speed*time_elapsed;
	}
	if(ready_for_pick) {
		double time_elapsed = giveCurrentTime() - pick_part.first_look_time;
		pick_part.current_pose.position.y = pick_part.first_pose.position.y - pick_part.speed*time_elapsed;
	}
	return;
}

void ConveyerParts::checkBoundaries() {
	if(allConveyerParts.size()!=0) {
		// double distance_from_end = allConveyerParts[0].current_pose.position.y - conveyer_end_y;
		// ROS_DEBUG_STREAM_THROTTLE(2, allConveyerParts[0].type << " is " << distance_from_end << "m away from end");
		// if(distance_from_end <= max_y_limit) {
		// 	// ROS_INFO_STREAM( allConveyerParts[0].type << " is out of limit now. Removing it. ");
		// 	allConveyerParts.erase(allConveyerParts.begin());
		// 	// ROS_INFO_STREAM_THROTTLE(2, "Number of conveyer parts avialbel now are " << allConveyerParts.size());
		// }

		if(allConveyerParts[0].current_pose.position.y <= max_y_limit) {
			// ROS_INFO_STREAM( allConveyerParts[0].type << " is out of limit now. Removing it. ");
			allConveyerParts.erase(allConveyerParts.begin());
			ROS_WARN_STREAM("--- out of reach ----" << allConveyerParts.size());
			// ROS_INFO_STREAM_THROTTLE(2, "Number of conveyer parts avialbel now are " << allConveyerParts.size());
		}
	}
	return;
}

void ConveyerParts::calculateSpeed() {
	double time_taken, distance;
	time_taken = current_detection.second_look_time - current_detection.first_look_time;
	distance = current_detection.first_pose.position.y - current_detection.second_pose.position.y;
 	if(time_taken!=0) {
		double speed = std::abs(distance/time_taken);
		current_detection.speed = round(speed*10)/10;  // Round off
		// ROS_INFO_STREAM("Speed of " << current_detection.type << " is set to " << current_detection.speed);
		return;
	}
	else {
		// ROS_WARN_STREAM("Time taken was zero. Cannot measure speed now.");
		return;
	}
}

ConveyerParts::ConveyerParts(ros::NodeHandle &node) {
	current_detection.emptyDetection();
	node_ = node;
	tf2_ros::TransformListener tfListener(tfBuffer_);
	ros::Duration timeout(5.0);
	bool found=false;
	double st = giveCurrentTime();
	
	while(!found) {
		try {
			C_to_W_transform = tfBuffer_.lookupTransform("world", "logical_camera_1_frame", ros::Time(0), timeout);
		}
		catch (tf2::TransformException &ex) {
			ROS_FATAL_STREAM( "Not able to find the conveyer camera frame -- " << ex.what());
			ros::Duration(1.0).sleep();
		}
		if(C_to_W_transform.child_frame_id.size()>0) found=true; 
	}
	Init();
}

geometry_msgs::Pose ConveyerParts::getPose_W(const geometry_msgs::Pose &pose_C) {
	geometry_msgs::Pose pose_now;
	tf2::doTransform(pose_C, pose_now, C_to_W_transform);
	// ROS_DEBUG_STREAM_THROTTLE(2," --- Pose in world --- " << " X : " << pose_now.position.x 
	// 									       << " Y : " << pose_now.position.y
	// 								 	       << " Z : " << pose_now.position.z);
	return pose_now;
}

void ConveyerParts::Init() {
	conveyer_subscriber = node_.subscribe("/ariac/logical_camera_1", 
											10,
											&ConveyerParts::conveyerLogicalCameraCallback,
											this);
	ROS_INFO_STREAM("Callback for conveyer logical camera created sucessfully");
}