/**
 *  Copyright 2020 Varun Asthana, Saumil Shah, Nalin Das, Markose Jacob, Aditya Goswami
 *  @file utils.h
 * 
 *  @author Varun Asthana
 *  @author Saumil Shah
 *  @author Nalin Das
 *  @author Markose Jacob
 *  @author Aditya Goswami
 * 
 *  @date 11/29/2020
 * 
 *  @brief Contains general utility typedefs
 * 
 *  @section DESCRIPTION
 *
 *  Header file for utility typedefs
 *
 */
#ifndef UTILS_H
#define UTILS_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <unordered_map>
#include <string>

#include <ros/ros.h>

#include <nist_gear/VacuumGripperState.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/**
 * @brief Shipment struct object
 */
typedef struct Shipment shipment; 
/**
 * @brief Object struct object
 */
typedef struct Order order;
/**
 * @brief Product struct object
 */
typedef struct Product product;

const double PI = 3.141592; // TODO correct!

/**
 * @brief Max Number of logical cameras
 */
const int MAX_NUMBER_OF_CAMERAS = 17;
/**
 * @brief Maximum picking attempts
 */
const int MAX_PICKING_ATTEMPTS = 3;
/**
 * @brief Above target z pos when picking/placing part
 */
const double ABOVE_TARGET = 0.1;
/**
 * @brief Pickup Timeout
 */
const double PICK_TIMEOUT = 4.0;
/**
 * @brief Retrieve Timeout
 */
const double RETRIEVE_TIMEOUT = 2.0;
/**
 * @brief Belt Speed in m/s
 */
const double BELT_SPEED = 0.2; 
/**
 * @brief Gripper Height
 */
const double GRIPPER_HEIGHT = 0.01;
/**
 * @brief Epsilon for the gripper to firmly touch
 */
const double EPSILON = 0.01;
/**
 * @brief Bin Height
 */
const double BIN_HEIGHT = 0.724;
/**
 * @brief Tray Height
 */
const double TRAY_HEIGHT = 0.755;
/**
 * @brief Rail Height
 */
const double RAIL_HEIGHT = 0.95;
/**
 * @brief Planning_Time for move_group
 */
const double PLANNING_TIME = 20; 
/**
 * @brief Max Exchange Attempts
 */
const int MAX_EXCHANGE_ATTEMPTS = 6; 
/**
 * @brief Action state name
 */
extern std::string action_state_name[];
/**
 * @brief Model height hashmap
 */
extern std::unordered_map<std::string, double> model_height;
/**
 * @brief Part states
 */
enum PartStates {FREE, BOOKED, UNREACHABLE, ON_TRAY, GRIPPED, GOING_HOME,
  REMOVE_FROM_TRAY, LOST};
/**
 * @brief Quaternion To Euler function
 * @param pose Quaternion pose
 * @return Converted Euler angles
 */
std::vector<double> quaternionToEuler(geometry_msgs::Pose pose);
/**
 * @brief Checks if aisle is clear
 * @param aisle_num Aisle number
 * @return True if Aisle clear else false
 */
bool isAisleClear(int aisle_num);
/**
 * @brief Preset Location typedef
 */
typedef struct PresetLocation {
    std::vector<double> gantry;
    std::vector<double> left_arm;
    std::vector<double> right_arm;
} start, bin3, agv1, agv2, flipped_pulley, conveyor_up;

/**
 * @brief Part typedef
 */
typedef struct Part {
  std::string type; // model type
  geometry_msgs::Pose pose; // world pose of the part
  int aisle_num = -1; //-1 if on the "bins SIDE", otherwise 1,2,3,4 (agv1 side =1, agv2 side =4)
  geometry_msgs::Pose save_pose; // pose of part in relative term
  std::string frame; // model frame (e.g., "logical_camera_1_frame")
  int camFrame;
  ros::Time time_stamp;
  std::string agv_id, id;
  std::vector<double> rpy_init;
  bool obstacle_free = true;
  PartStates state; // model state (enum PartStates)
  float yaw_correction= 0;
  bool flip_part = false;
  bool flip_part_correction = false;
  bool flip_part_preset_correction = false;
} part;
/**
 * @brief Position typedef
 */
typedef struct Position {
    std::vector<double> gantry;
    std::vector<double> left;
    std::vector<double> right;
} position;
/**
 * @brief Shipment typedef
 */
typedef struct Shipment {
    std::string shipment_type;
    std::string agv_id;
    std::vector<Product> products;
    int prodComplete = 0;
    order* parent_order;
    int parent_order_idx;
} shipment;
/**
 * @brief Product typedef
 */
typedef struct Product {
    std::string type;
    geometry_msgs::Pose pose; //relative pose in tray
    geometry_msgs::Pose agv_world_pose;
    part p; // NEW here!
    // std::string frame_of_origin;
    // geometry_msgs::Pose actual_pose;
    // std::string actual_pose_frame;
    geometry_msgs::Pose estimated_conveyor_pose; //pose for robot in world frame- wait at it with gripper activated
    std::string agv_id;
    std::string tray;
    std::string arm_name;
    std::string cache_id;
    shipment* parent_shipment;
    bool high_priority;
    int correction_attempts;
    int service_attempts;
    bool part_placed = false;
    bool mv_prod = false;
    int shipId;
    std::vector<double> rpy_final;
    float yaw_correction= 0;

    // Product(); // contructor
} product;
/**
 * @brief Order typedef
 */
typedef struct Order {
    std::string order_id;
    std::vector<Shipment> shipments;
} order;
/**
 * @brief Stats typedef
 */
typedef struct Stats {
  double total_time = 0.0;
  double fail_time = 0.0;
  int calls = 0;
  int fails = 0;
} stats;
/**
 * @brief All orders struct
 */
struct all_Order{
    Product prod;
    int ship_num;
    std::string shipment_type;
    bool priority = false;
    struct all_Order *next;
};
/**
 * @brief AGV Information struct
 */
struct agvInfo{
    std::unordered_map<std::string, std::vector<Product>> prod_on_tray;
    std::vector<struct all_Order *> complete_order_data;
    int count = 0;
};

#endif