#include <algorithm>
#include <vector>
#include <string>
#include <unordered_map>
#include <geometry_msgs/Pose.h>
#include <nist_gear/Proximity.h>
#include "competition.h"
#include <ros/ros.h>
#include "obstacles.h"


// static double giveCurrentTime() {
// 	return ros::Time::now().toSec();
// }

// double round (double x) {
//     return std::floor(x + 0.5f);
// }

ObstaclesInAisle::ObstaclesInAisle(ros::NodeHandle& node) {
	node_ = node;
	aisle_1_dir.first = 0;
	aisle_1_dir.second = 0;

    aisle_2_dir.first = 0;
	aisle_2_dir.second = 0;

	aisle_3_dir.first = 0;
	aisle_3_dir.second = 0;

	aisle_4_dir.first = 0;
	aisle_4_dir.second = 0;

    for(int i = 0; i < 6; ++i){
    	aisle_clear.push_back(true);
    	aisle_1_sensor_data.push_back(0);
    	aisle_2_sensor_data.push_back(0);
    	aisle_3_sensor_data.push_back(0);
    	aisle_4_sensor_data.push_back(0);
    }
    Init();
}


bool ObstaclesInAisle::isAisleClear(int aisle_num){
    if(aisle_num == -1){
        return true;
    }
    return aisle_clear[aisle_num];
}

void ObstaclesInAisle::breakbeam_callback(const nist_gear::Proximity::ConstPtr &msg, int id) {
	int aisle_num = id/10;
	std::vector<int>* aisle = NULL;
	std::pair<int, int>* aisle_dir = NULL;
	switch(aisle_num){
		case 1: aisle = &aisle_1_sensor_data;
				aisle_dir = &aisle_1_dir;
				break;
		case 2: aisle = &aisle_2_sensor_data;
				aisle_dir = &aisle_2_dir;
				break;
		case 3: aisle = &aisle_3_sensor_data;
				aisle_dir = &aisle_3_dir;
				break;
		case 4: aisle = &aisle_4_sensor_data;
				aisle_dir = &aisle_4_dir;
	}

	int sensor_num = id % 10;
	if(msg->object_detected){
		aisle_clear[aisle_num] = false;
		(*aisle)[sensor_num] = !(*aisle)[sensor_num];
		if(sensor_num !=0 && sensor_num !=5 && id !=(*aisle_dir).first ){
			(*aisle_dir).second = id > (*aisle_dir).first ? 0 : 1;  // 0 = moving from conveyor to edge
																	// 1 = moving from edge to conveyor
		}
		(*aisle_dir).first = id;
	}

	return;
}

bool ObstaclesInAisle::moveBot(geometry_msgs::Pose pose, int gapNum, int aisle_num, float currX, int currGap){
	std::pair<int, int>* aisle_dir = NULL;
	switch(aisle_num){
		case 1: aisle_dir = &aisle_1_dir;
				break;
		case 2: aisle_dir = &aisle_2_dir;
				break;
		case 3: aisle_dir = &aisle_3_dir;
				break;
		case 4: aisle_dir = &aisle_4_dir;
	}

	// 0 = moving from conveyor to edge
	// 1 = moving from edge to conveyor
	int direction = 0;
	int negate = 1;
	if(pose.position.x > currX){
		direction = 1;
		negate = -1;
	}
	ROS_WARN_STREAM("Desired direction = " << direction);
	// if((*aisle_dir).first %10 ==0 || (*aisle_dir).first %10 ==5){
	// 	return false;
	// }
	if((*aisle_dir).first * negate >= negate*(aisle_num*10 + gapNum) && (*aisle_dir).second == direction){
		ROS_FATAL_STREAM("Found True");
		ROS_WARN_STREAM("Gap num = " << (*aisle_dir).first);
		ROS_WARN_STREAM("Motion= " << (*aisle_dir).second);
		return true;
	}else if((*aisle_dir).first * negate *(-1) >= negate*(-1)*(aisle_num*10 + currGap) && (*aisle_dir).second != direction){
		ROS_FATAL_STREAM("Else True");
		ROS_WARN_STREAM("Gap num = " << (*aisle_dir).first);
		ROS_WARN_STREAM("Motion= " << (*aisle_dir).second);
		return true; 
	}
	return false;
}

void ObstaclesInAisle::Init() {
	breakbeam_subscriber_1.resize(breakbeam_sensor_topics_1.size());
	breakbeam_subscriber_2.resize(breakbeam_sensor_topics_2.size());
	breakbeam_subscriber_3.resize(breakbeam_sensor_topics_3.size());
	breakbeam_subscriber_4.resize(breakbeam_sensor_topics_4.size());

	for(int i=0; i<breakbeam_sensor_topics_1.size(); i++) {
		breakbeam_subscriber_1[i] = node_.subscribe<nist_gear::Proximity>( breakbeam_sensor_topics_1[i], 1, 
																			boost::bind(&ObstaclesInAisle::breakbeam_callback, this, _1, i+10));
	}
	
	for(int i=0; i<breakbeam_sensor_topics_2.size(); i++) {
		breakbeam_subscriber_2[i] = node_.subscribe<nist_gear::Proximity>( breakbeam_sensor_topics_2[i], 1,
																			boost::bind(&ObstaclesInAisle::breakbeam_callback, this, _1, i+20));
 	}

    for(int i=0; i<breakbeam_sensor_topics_3.size(); i++) {
    	breakbeam_subscriber_3[i] = node_.subscribe<nist_gear::Proximity>( breakbeam_sensor_topics_3[i], 1,
    																		boost::bind(&ObstaclesInAisle::breakbeam_callback, this, _1, i+30));
    }

    for(int i=0; i<breakbeam_sensor_topics_4.size(); i++) {
    	breakbeam_subscriber_4[i] = node_.subscribe<nist_gear::Proximity>( breakbeam_sensor_topics_4[i], 1,
    																		boost::bind(&ObstaclesInAisle::breakbeam_callback, this, _1, i+40));
    }
	
	ros::Duration(5.0).sleep();

	for(int i = 0; i< aisle_clear.size(); ++i){
		std::cout << "\nAisle " << i << " : " << aisle_clear[i];
		if(aisle_clear[i] == false){
			num_obstacles++;
		}
	}

	std::cout<<"\n Total obstacles = " << num_obstacles << std::endl;
}