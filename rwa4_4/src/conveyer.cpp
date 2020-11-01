#include <algorithm>
#include <vector>
#include <string>
#include <unordered_map>
#include "competition.h"
#include <ros/ros.h>
#include "conveyer.h"

void Detection::emptyDetection() {
	part_set = false;
	first_look_time = -1;
	second_look_time = -1;
	speed = -1;
	type = "";
}

static double giveCurrentTime() {
	return ros::Time::now().toSec();
}

void ConveyerParts::conveyerLogicalCameraCallback(const nist_gear::LogicalCameraImage& msg) {
	// No part detected
	if(msg.models.size()!=1 && current_detection.type.size()==0) {
		// ROS_DEBUG_STREAM("No parts detected on conveyer belt camera");
		return;
	}
	current_pose = getPose_W(msg.models[0].pose);
	// When seen for the first time set the name, first seen time and first seen position
	if(msg.models.size()==1 && current_detection.type.size()==0) {
		if(current_pose.position.y < part_read_limit) {
			ROS_WARN_STREAM("Not taking in cosideration " << current_pose.position.y);
			return;
		} 
		current_detection.type = msg.models[0].type;  // Setting name of the part if not assigned
		ROS_DEBUG_STREAM("Product name on conveyer set to " << current_detection.type);
	}
	// First pose and time detection
	if(current_detection.first_look_time == -1) {
		current_detection.first_look_time = giveCurrentTime();
		ROS_DEBUG_STREAM("Product on conveyer was watched first at " << current_detection.first_look_time << " seconds");
		current_detection.first_pose = getPose_W(msg.models[0].pose);
		return;
	}
	// Second pose and time and speed
	if(current_detection.second_look_time == -1) {
		double present_time = giveCurrentTime();
		// Too soon and close
		if(present_time - current_detection.first_look_time <= 0.01) {
			return;
		}
		current_detection.second_look_time = giveCurrentTime();
		ROS_DEBUG_STREAM("Product on conveyer was watched second time at " << current_detection.second_look_time << " seconds");
		current_detection.second_pose = getPose_W(msg.models[0].pose);
		calculateSpeed();
		current_detection.part_set = true;
		allConveyerParts.emplace_back(current_detection);
		return;
	}
	updateCurrentPoses();
	checkBoundaries();
	checkCurrentPartSet();
}

void ConveyerParts::checkCurrentPartSet() {
	if(current_detection.part_set && current_detection.current_pose.position.y < part_read_limit) {
		ROS_DEBUG_STREAM( current_detection.type << " is well set. Removing data from class");
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

bool ConveyerParts::giveClosestPart(const std::string &part_name, geometry_msgs::Pose &poseOnConveyer) {
	for(int i=0; i<allConveyerParts.size(); i++) {
		if(allConveyerParts[i].type.compare(part_name) == 0) {
			ROS_INFO_STREAM("Found the part on conveyer");
			poseOnConveyer = allConveyerParts[i].current_pose;
			poseOnConveyer.position.y -= offset;
			ROS_INFO_STREAM("This pose would be great to pick up a part from conveyer "
							<< " X : " << poseOnConveyer.position.x
							<< " Y : " << poseOnConveyer.position.y
							<< " Z : " << poseOnConveyer.position.z);
			allConveyerParts.erase(allConveyerParts.begin()+i);
			return true;
		}
	}
	return false;
}

void ConveyerParts::updateCurrentPoses() {
	for(auto &part: allConveyerParts) {
		double time_elapsed = giveCurrentTime() - part.first_look_time;
		part.current_pose.position.y = part.first_pose.position.y - part.speed*time_elapsed;
		ROS_DEBUG_STREAM("New position of " << part.type << " is " 
							<< " X : " << part.current_pose.position.x
							<< " Y : " << part.current_pose.position.y
							<< " Z : " << part.current_pose.position.z);
	}
	return;
}

void ConveyerParts::checkBoundaries() {
	if(allConveyerParts.size()!=0) {
		double distance_from_end = std::abs(allConveyerParts[0].current_pose.position.y - conveyer_end_y);
		ROS_DEBUG_STREAM(allConveyerParts[0].type << " is far at " << distance_from_end);
		if(distance_from_end <= max_y_limit) {
			ROS_INFO_STREAM( allConveyerParts[0].type << " is out of limit now. Removing it. ");
			allConveyerParts.erase(allConveyerParts.begin());
		}
	}
	return;
}

void ConveyerParts::calculateSpeed() {
	double time_taken, distance;
	time_taken = current_detection.second_look_time - current_detection.first_look_time;
	distance = current_detection.first_pose.position.y - current_detection.second_pose.position.y;
 	if(time_taken!=0) {
		current_detection.speed = std::abs(distance/time_taken);
		ROS_DEBUG_STREAM("Speed is set to " << current_detection.speed);
		return;
	}
	else {
		ROS_WARN_STREAM("Time taken was zero. Cannot measure speed now.");
		return;
	}
}

ConveyerParts::ConveyerParts(ros::NodeHandle &node) {
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
			ROS_FATAL_STREAM( "Nogt able to find the conveyer camera frame -- " << ex.what());
			ros::Duration(1.0).sleep();
		}
		if(C_to_W_transform.child_frame_id.size()>0) found=true; 
	}
	ROS_INFO_STREAM("------ C_W ------\n" << " X : " << C_to_W_transform.transform.translation.x 
										  << " Y : " << C_to_W_transform.transform.translation.y
										  << " Z : " << C_to_W_transform.transform.translation.z);
	Init();
}

geometry_msgs::Pose ConveyerParts::getPose_W(const geometry_msgs::Pose &pose_C) {
	geometry_msgs::Pose pose_now;
	tf2::doTransform(pose_C, pose_now, C_to_W_transform);
	ROS_DEBUG_STREAM(" --- Pose in world --- " << " X : " << pose_now.position.x 
										       << " Y : " << pose_now.position.y
									 	       << " Z : " << pose_now.position.z);
	return pose_now;
}

void ConveyerParts::Init() {
	conveyer_subscriber = node_.subscribe("/ariac/logical_camera_1", 
											10,
											&ConveyerParts::conveyerLogicalCameraCallback,
											this);
	ROS_INFO_STREAM("Callback for conveyer logical camera created sucessfully");
}