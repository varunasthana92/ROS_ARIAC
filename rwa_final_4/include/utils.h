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

typedef struct Shipment shipment; // forward declarations
typedef struct Order order;
typedef struct Product product;

const double PI = 3.141592; // TODO correct!

// Logical cameras
const int MAX_NUMBER_OF_CAMERAS = 17;



const int MAX_PICKING_ATTEMPTS = 3; // for pickup
const double ABOVE_TARGET = 0.1; // above target z pos when picking/placing part
const double PICK_TIMEOUT = 4.0;
const double RETRIEVE_TIMEOUT = 2.0;

const double BELT_SPEED = 0.2; // m/s

const double GRIPPER_HEIGHT = 0.01;
const double EPSILON = 0.018; // for the gripper to firmly touch

const double BIN_HEIGHT = 0.724;
const double TRAY_HEIGHT = 0.755;
const double RAIL_HEIGHT = 0.95;

const double PLANNING_TIME = 20; // for move_group
const int MAX_EXCHANGE_ATTEMPTS = 6; // Pulley flip

extern std::string action_state_name[];
extern std::unordered_map<std::string, double> model_height;

enum PartStates {FREE, BOOKED, UNREACHABLE, ON_TRAY, GRIPPED, GOING_HOME,
  REMOVE_FROM_TRAY, LOST};

std::vector<double> quaternionToEuler(geometry_msgs::Pose pose);
bool isAisleClear(int aisle_num);

typedef struct PresetLocation {
    std::vector<double> gantry;
    std::vector<double> left_arm;
    std::vector<double> right_arm;
} start, bin3, agv1, agv2, flipped_pulley, conveyor_up;


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
} part;

typedef struct Position {
    std::vector<double> gantry;
    std::vector<double> left;
    std::vector<double> right;
} position;

typedef struct Shipment {
    std::string shipment_type;
    std::string agv_id;
    std::vector<Product> products;
    int prodComplete = 0;
    order* parent_order;
    int parent_order_idx;
} shipment;

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

typedef struct Order {
    std::string order_id;
    std::vector<Shipment> shipments;
} order;

typedef struct Stats {
  double total_time = 0.0;
  double fail_time = 0.0;
  int calls = 0;
  int fails = 0;
} stats;

struct agvInfo{
    std::unordered_map<std::string, std::vector<Product>> prod_on_tray;
    int count = 0;
};

#endif