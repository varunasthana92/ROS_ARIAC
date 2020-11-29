/**
 *  Copyright 2020 Nalin Das
 *  @file conveyor.h
 * 
 *  @author Varun Asthana
 *  @author Saumil Shah
 *  @author Nalin Das
 *  @author Markose Jacob
 *  @author Aditya Goswami
 * 
 *  @date 11/28/2020
 * 
 *  @brief Detection and conveyor parts class
 * 
 *  @section DESCRIPTION
 *
 *  Header file for the detection and conveyor class
 *
 */
#ifndef CONVEYER_H
#define CONVEYER_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <unordered_map>
#include <string>
#include <queue>
#include <ros/ros.h>
#include <nist_gear/LogicalCameraImage.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


// const double PI = 3.141592; // TODO correct!

/**
 * @brief Detection class
 */
class Detection {
  public:
    /**
     * @brief Part set variable
     */
    bool part_set = false;
    /**
     * @brief Picked up variable
     */
    bool picked_up = false;
    /**
     * @brief First look time variable
     */
    double first_look_time=-1;
    /**
     * @brief Second look time variable
     */
    double second_look_time=-1;
    /**
     * @brief Speed variable
     */
    double speed=-1;
    /**
     * @brief Type variable
     */
    std::string type="";
    /**
     * @brief Pose 1 variable
     */
    geometry_msgs::Pose first_pose;
    /**
     * @brief Pose 2 variable
     */
    geometry_msgs::Pose second_pose;
    /**
     * @brief Current pose variable
     */
    geometry_msgs::Pose current_pose;
    /**
     * @brief Resets detection parameters
     * @param None
     * @return None
     */
    void emptyDetection();
};

/**
 * @brief Conveyor parts class
 */
class ConveyerParts {
  public:
    /**
     * @brief Conveyer subscriber variable
     */
    ros::Subscriber conveyer_subscriber;
    /**
     * @brief Current detection variable
     */
    Detection current_detection;
    /**
     * @brief All Conveyer Parts variable
     * @param None
     * @return None
     */
    std::vector<Detection> allConveyerParts;
    /**
     * @brief Conveyor logical camera callback
     * @param msg Logical camera image
     * @return None
     */
    void conveyerLogicalCameraCallback(const nist_gear::LogicalCameraImage& msg);
    /**
     * @brief Constructor
     * @param node ROS nodehandle
     * @return None
     */
    ConveyerParts(ros::NodeHandle& node);
    /**
     * @brief Initialization 
     * @param None
     * @return None
     */
    void Init();
    /**
     * @brief Checks for part
     * @param part_name Part name
     * @return None
     */
    bool checkPart(const std::string &part_name);
    /**
     * @brief Estimated time
     */
    double estimated_time=0;
    /**
     * @brief Gets closest part
     * @param part_name Part name
     * @param poseOnConveyer Current pose of part on conveyor
     * @return True/False
     */
    bool giveClosestPart(const std::string &part_name, geometry_msgs::Pose &poseOnConveyer);  // Will give the current location of the part on conveyer 
    /**
     * @brief Checks if part ready to pickup
     * @param None
     * @return True if ready to be picked up else false
     */
    bool checkForPick();
  private:
    /**
     * @brief Part read limit variable
     */
    double part_read_limit=3.0;
    /**
     *  @brief Max y limit variable
     */
    double max_y_limit = -1.5;
    /**
     *  @brief Conveyer end y variable
     */
    double conveyer_end_y=-2;
    /**
     *  @brief Offset variable
     */
    double offset = 1.8;
    /**
     *  @brief Pick pose variable
     */
    geometry_msgs::Pose pick_pose;
    /**
     *  @brief Pick part variable
     */
    Detection pick_part;
    /**
     *  @brief Ready for pick variable
     */
    bool ready_for_pick=false;
    /**
     *  @brief Gets the pose in W frame
     *  @param pose_C Pose in c frame
     *  @return Pose in W frame
     */
    geometry_msgs::Pose getPose_W(const geometry_msgs::Pose &pose_C);
    /**
     *  @brief ROS Nodehandle
     */
    ros::NodeHandle node_;
    /**
     *  @brief Start time variable
     */
    double start_time = 0;
    /**
     *  @brief Velocity variable
     */
    double velocity = 0;
    /**
     *  @brief tfBuffer 
     */
    tf2_ros::Buffer tfBuffer_;
    /**
     *  @brief Adds new part to map
     *  @param detection Detection variable
     *  @return None
     */
    void addNewPartToMap(Detection detection); 
    /**
     *  @brief C to W transform variable
     */
    geometry_msgs::TransformStamped C_to_W_transform;
    /**
     *  @brief Calculates the detection speed
     *  @param None None
     *  @return None
     */
    void calculateSpeed();
    /**
     *  @brief Updates the current poses
     *  @param None None
     *  @return None
     */
    void updateCurrentPoses();
    /**
     *  @brief Checks the boundaries
     *  @param None None
     *  @return None
     */
    void checkBoundaries();
    /**
     *  @brief Checks if current part is set
     *  @param None None
     *  @return None
     */
    void checkCurrentPartSet();
};

#endif