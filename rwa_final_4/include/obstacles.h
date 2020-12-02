/**
 *  Copyright 2020 Varun Asthana, Saumil Shah, Nalin Das, Markose Jacob, Aditya Goswami
 *  @file obstacles.h
 * 
 *  @author Varun Asthana
 *  @author Saumil Shah
 *  @author Nalin Das
 *  @author Markose Jacob
 *  @author Aditya Goswami
 * 
 *  @date 11/29/2020
 * 
 *  @brief ObstaclesInAisle class
 * 
 *  @section DESCRIPTION
 *
 *  Header file for the ObstaclesInAisle class
 *
 */
#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <nist_gear/Proximity.h>
#include <unordered_map>

// const double PI = 3.141592; // TODO correct!



// node.subscribe<nist_gear::Proximity>(breakbeam_sensor_topics[i], 10, \
//             boost::bind(&GantryControl::breakbeam_sensor_callback, &gantry, _1, i+1));
/**
 * @brief ObstaclesInAisle class
 */
class ObstaclesInAisle {
  private:
    /**
     * @brief Breakbeam subscriber 1
     */
    std::vector<ros::Subscriber> breakbeam_subscriber_1;
    /**
     * @brief Breakbeam subscriber 2
     */
    std::vector<ros::Subscriber> breakbeam_subscriber_2;
    /**
     * @brief Breakbeam subscriber 3
     */
    std::vector<ros::Subscriber> breakbeam_subscriber_3;
    /**
     * @brief Breakbeam subscriber 4
     */
    std::vector<ros::Subscriber> breakbeam_subscriber_4;
    /**
     * @brief Breakbeam sensor topics 1
     */
    std::vector<std::string> breakbeam_sensor_topics_1{
      "/ariac/breakbeam_10",
      "/ariac/breakbeam_11",
      "/ariac/breakbeam_12",
      "/ariac/breakbeam_13",
      "/ariac/breakbeam_14",
      "/ariac/breakbeam_15"
      };
    /**
     * @brief Breakbeam sensor topics 2
     */
    std::vector<std::string> breakbeam_sensor_topics_2{
      "/ariac/breakbeam_20",
      "/ariac/breakbeam_21",
      "/ariac/breakbeam_22",
      "/ariac/breakbeam_23",
      "/ariac/breakbeam_24",
      "/ariac/breakbeam_25"
      };
    /**
     * @brief Breakbeam sensor topics 3
     */
    std::vector<std::string> breakbeam_sensor_topics_3{
      "/ariac/breakbeam_30",
      "/ariac/breakbeam_31",
      "/ariac/breakbeam_32",
      "/ariac/breakbeam_33",
      "/ariac/breakbeam_34",
      "/ariac/breakbeam_35"
      };
    /**
     * @brief Breakbeam sensor topics 4
     */
    std::vector<std::string> breakbeam_sensor_topics_4{
      "/ariac/breakbeam_40",
      "/ariac/breakbeam_41",
      "/ariac/breakbeam_42",
      "/ariac/breakbeam_43",
      "/ariac/breakbeam_44",
      "/ariac/breakbeam_45"
      };
  public:
    /**
     * @brief Number of obstacles 
     */
    int num_obstacles = 0;
    /**
     * @brief Aisle 1 sensor data
     */
    std::vector<int> aisle_1_sensor_data;
    /**
     * @brief Aisle 2 sensor data 
     */
    std::vector<int> aisle_2_sensor_data;
    /**
     * @brief Aisle 3 sensor data 
     */
    std::vector<int> aisle_3_sensor_data;
    /**
     * @brief Aisle 4 sensor data 
     */
    std::vector<int> aisle_4_sensor_data;
    /**
     * @brief aisle_1_dir 
     */
    std::pair<int, int> aisle_1_dir;
    /**
     * @brief Aisle 2 dir
     */
    std::pair<int, int> aisle_2_dir;
    /**
     * @brief Aisle 3 dir
     */
    std::pair<int, int> aisle_3_dir;
    /**
     * @brief Aisle 4 dir
     */
    std::pair<int, int> aisle_4_dir;
    /**
     * @brief Aisle clear
     */
    std::vector<bool> aisle_clear;
    /**
     * @brief ROS Nodehandle
     */
    ros::NodeHandle node_;
    /**
     * @brief Breakbeam callback
     * @param msg Proximity sensor data
     * @param id Sensor id number
     * @return None
     */
    void breakbeam_callback(const nist_gear::Proximity::ConstPtr &msg, int id);
    /**
     * @brief Move Robot
     * @param destX Destination X-Coordinate
     * @param gapNum Gap number
     * @param aisle_num Aisle number
     * @param currX Current X-Coordinate
     * @param currGap Current gap
     * @return True if moved successfully else false
     */
    bool moveBot(float destX, int gapNum, int aisle_num, float currX, int currGap);
    /**
     * @brief Constructor
     * @param node ROS nodehandle
     * @return None
     */
    ObstaclesInAisle(ros::NodeHandle& node); 
    /**
     * @brief Checks if aisle is clear
     * @param aisle_num Aisle number
     * @return True if Aisle clear else false
     */
    bool isAisleClear(int aisle_num);
    /**
     * @brief Initialization 
     * @param None None
     * @return None
     */
    void Init();
  // private:
};

#endif