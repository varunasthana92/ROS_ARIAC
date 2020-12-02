/**
 *  Copyright 2020 Nalin Das
 *  @file competition.h
 * 
 *  @author Varun Asthana
 *  @author Saumil Shah
 *  @author Nalin Das
 *  @author Markose Jacob
 *  @author Aditya Goswami
 * 
 *  @date 11/28/2020
 * 
 *  @brief Starts the ARIAC competition
 * 
 *  @section DESCRIPTION
 *
 *  Header file containing ARIAC competition class
 *
 */
#ifndef COMPETITION_H
#define COMPETITION_H

#include <vector>

#include <ros/ros.h>
#include <stdio.h> 
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Clock.h>
#include <nist_gear/Order.h>

#include "utils.h"


/**
 * @brief Competition class
 * 
 */
class Competition
{
public:
    /**
     * @brief Constructor
     * @param node ROS nodehandle
     * @return None
     */
    explicit Competition(ros::NodeHandle & node);
    /**
     * @brief Initialization function
     * @param None
     * @return None
     */
    void init();
    /**
     * @brief Starts the competition
     * @param None
     * @return None
     */
    void startCompetition();
    /**
     * @brief Ends the competition
     * @param None
     * @return None
     */
    void endCompetition();
    /**
     * @brief Sends AGV to complete order
     * @param agv AGV Number
     * @param ship_type Shipment type
     * @return None
     */
    void shipAgv(std::string agv, std::string ship_type);
    /**
     * @brief Competition state callback
     * @param msg Contains competition state
     * @return None
     */
    void competition_state_callback(const std_msgs::String::ConstPtr & msg);
    /**
     * @brief Competition clock callback
     * @param msg Contain simulation time elapsed
     * @return None
     */
    void competition_clock_callback(const rosgraph_msgs::Clock::ConstPtr & msg);
    /**
     * @brief Order callback
     * @param msg Contains order number
     * @return None
     */
    void order_callback(const nist_gear::Order::ConstPtr & msg);
    /**
     * @brief Gets the competition time elapsed
     * @param None
     * @return Time elapsed
     */
    double getClock();
    /**
     * @brief Gets the start time
     * @param None
     * @return Competition start time
     */
    double getStartTime();
    /**
     * @brief Gets the competition state
     * @param None
     * @return Competition state
     */
    std::string getCompetitionState();
    /**
     * @brief Gets the competition stats
     * @param function Function
     * @return Competition stats
     */
    stats getStats(std::string function);
    /**
     * @brief Gets the AGV shipment data
     * @param None
     * @return AGV Shipment Data
     */
    std::unordered_map< std::string, std::string> agv_ship_data;

private:
    /**
    *  @brief ROS Nodehandle
     */
    ros::NodeHandle node_;
    /**
    *  @brief Competition state
     */
    std::string competition_state_;
    /**
    *  @brief Current score
     */
    double current_score_;
    /**
    *  @brief Competition clock
     */
    ros::Time competition_clock_;
    /**
    *  @brief Competition start time
     */
    double competition_start_time_; 
    /**
    *  @brief Current score subscriber
     */
    ros::Subscriber current_score_subscriber_;
    /**
    *  @brief Competition state subscriber
     */
    ros::Subscriber competition_state_subscriber_;
    /**
    *  @brief Competition clock subscriber
     */
    ros::Subscriber competition_clock_subscriber_;
    /**
    *  @brief Orders subscriber
     */
    ros::Subscriber orders_subscriber_;
    /**
    *  @brief Received orders
     */
    std::vector<nist_gear::Order> received_orders_;
    /**
    *  @brief Collects stats
     */
    stats init_;
};

#endif
