/**
 *  Copyright 2020 Nalin Das
 *  @file gantry_control.h
 * 
 *  @author Varun Asthana
 *  @author Saumil Shah
 *  @author Nalin Das
 *  @author Markose Jacob
 *  @author Aditya Goswami
 * 
 *  @date 11/28/2020
 * 
 *  @brief Gantry class
 * 
 *  @section DESCRIPTION
 *
 *  Header file for the Gantry Control class
 *
 */
#ifndef GANTRYCONTROL_H
#define GANTRYCONTROL_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <array>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/VacuumGripperControl.h>

#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "utils.h"
#include "orderBuild.h"
#include "conveyer.h"
#include "obstacles.h"
#include <nist_gear/LogicalCameraImage.h>

/**
 * @brief Gantry Control class
 */
class GantryControl {

  public:
    /**
     * @brief Constructor
     * @param node ROS nodehandle
     * @return None
     */
    GantryControl(ros::NodeHandle & node);
    /**
     * @brief Initialization 
     * @param None
     * @return None
     */
    void init();
    /**
     * @brief Gets the stats 
     * @param function Function
     * @return Stats
     */
    stats getStats(std::string function);
    /**
     * @brief Quality camera 1 callback 
     * @param msg Logical Camera Image
     * @return None
     */
    void qualityCallback1(const nist_gear::LogicalCameraImage& msg);
    /**
     * @brief Quality camera 2 callback  
     * @param msg Logical Camera Image
     * @return None
     */
    void qualityCallback2(const nist_gear::LogicalCameraImage& msg);
    /**
     * @brief Makes robot pick the part 
     * @param part Part
     * @return True if picked successfully else false
     */
    bool pickPart(part part);
    /**
     * @brief Makes robot place the part  
     * @param product Product
     * @param agv AGV Number
     * @param arm Left/Right arm
     * @param cusmomised product data from order
     * @param class object of type ConveyerParts
     * @return True if placed successfully else false
     */
    bool placePart(product &product, std::string agv, std::string arm, struct all_Order *curr_prod, ConveyerParts &conveyerPartsObj);
    /**
     * @brief Send command message to robot controller 
     * @param command_msg Joint trajectory command message
     * @return True if command published successfully else false
     */
    bool send_command(trajectory_msgs::JointTrajectory command_msg);
    /**
     * @brief Makes robot move to preset location 
     * @param location Preset location
     * @return None
     */
    void goToPresetLocation(PresetLocation location);
    /**
     * @brief Rotate gantry 
     * @param angle Angle
     * @return None
     */
    void rotate_gantry(double angle);
    /**
     * @brief Activate gripper 
     * @param gripper_id Gripper ID
     * @return None
     */
    void activateGripper(std::string gripper_id);
    /**
     * @brief deactivateGripper 
     * @param gripper_id Gripper ID
     * @return None
     */
    void deactivateGripper(std::string gripper_id);
    /**
     * @brief Logical camera 16 callback 
     * @param msg LogicalCameraImage
     * @return None
     */
    void logicalCallback16(const nist_gear::LogicalCameraImage& msg);
    /**
     * @brief Logical camera 17 callback 
     * @param msg LogicalCameraImage
     * @return None
     */
    void logicalCallback17(const nist_gear::LogicalCameraImage& msg);
    /**
     * @brief Pick from conveyor 
     * @param product Product
     * @param conveyerPartsObj Conveyor parts object
     * @return Bool
     */
    bool pickFromConveyor(Product &product, ConveyerParts &conveyerPartsObj);
    /**
     * @brief Pose Matches 
     * @param pose1 Pose 1
     * @param pose2 Pose 2
     * @return True if pose matches else false
     */
    bool poseMatches(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2);
    /**
     * @brief Check if a part exists on AGV 
     * @param name Name
     * @param part_pose Part pose
     * @param agv AGV Info
     * @return True if part exists on AGV else false
     */
    bool check_exist_on_agv(const std::string &name, const geometry_msgs::Pose &part_pose, agvInfo &agv);
    /**
     * @brief Flip part 
     * @param None None
     * @return None
     */
    void flipPart();
    /**
     * @brief Moves robot to start position 
     * @param x X-Coordinate
     * @param y Y-Coordinate
     * @param left_arm Left arm 
     * @return None
     */
    bool move2start ( float x, float y, std::vector<double> left_arm);
    /**
     * @brief Move to target 
     * @param x X-Coordinate
     * @param y Y-Coordinate
     * @param gantryX Gantry position X coordinate
     * @param gantryY Gantry position Y coordinate
     * @param currGap Current gap
     * @param left_arm Left arm 
     * @return Joint angles of arm
     */
    std::vector<double> move2trg (float x, float y, float &gantryX, float &gantryY, int currGap, std::vector<double> left_arm);
    /**
     * @brief Move to closest gap 
     * @param part Part
     * @param shelfGaps Shelf gaps
     * @param gapNum Gap number
     * @param actPart Act part
     * @param gantryX Gantry X-Coordinate
     * @param gantryY Gantry Y-Coordinate
     * @param obj ObstaclesInAisle object
     * @param newGap New gap
     * @return True if moved successfuly else false
     */
    bool move2closestGap(struct Part &part, std::vector< std::pair<float , float> > &shelfGaps,
                        const std::vector<int> &gapNum, bool actPart, float &gantryX, float &gantryY,
                        ObstaclesInAisle &obj, int &newGap);
    /**
     * @brief Gets the nearest gap from shelf
     * @param destX Destination X-Coordinate
     * @param aisle_num Aisle number
     * @param actPart Act part
     * @param obstObj ObstaclesInAisle object
     * @param shelfGaps Gap from shelf
     * @return Nearest gap from shelf
     */
    int getNearestGap(float destX, int aisle_num, bool actPart, ObstaclesInAisle &obstObj,
                                const std::vector< std::pair<float , float> > &shelfGaps);
    /**
     * @brief Makes robot escape from Aisle 
     * @param aisle_num Aisle number
     * @param shelfGaps Gap from shelves
     * @param gapNum Gap number
     * @param actPart Act part
     * @param gantryX Gantry X-Coordinate
     * @param gantryY Gantry Y-Coordinate
     * @param obstObj ObstaclesInAisle object
     * @param newGap Newgap from shelf
     * @param left_arm Left arm 
     * @param pickStatus Status of pickup
     * @return True if escape successfull else false
     */
    bool escape(int &aisle_num, std::vector< std::pair<float , float> > &shelfGaps, const std::vector<int> &gapNum,
                bool actPart, float &gantryX, float &gantryY, ObstaclesInAisle &obstObj, int &newGap,
                std::vector<double> &left_arm, bool pickStatus);
    /**
     * @brief Clear the AGV
     * @param agv AGV Number
     * @param buildObj BuildClass object
     * @return None
     */
    void clearAgv(std::string agv, BuildClass &buildObj);
    /**
     * @brief Gets the robot pose 
     * @param None None
     * @return Robot pose
     */
    geometry_msgs::Pose getRobotPose(){
      return full_robot_group_.getCurrentPose().pose;

    }
    /**
     * @brief Gets the gripper state 
     * @param arm_name Arm name
     * @return Gripper state
     */
    nist_gear::VacuumGripperState getGripperState(std::string arm_name);
    /**
     * @brief Gets target pose in world frame 
     * @param target Target
     * @param agv AGV Number
     * @param arm Left/Right arm
     * @return Target pose in world frame
     */
    geometry_msgs::Pose getTargetWorldPose(geometry_msgs::Pose target, std::string agv, std::string arm);
    /**
     * @brief Quality call count variable
     */
    int quality_call_count =0;
    /**
     * @brief Start preset location
     */
    start start_;
    /**
     * @brief Bin 3 preset location 
     */
    bin3 bin3_;
    /**
     * @brief Flipped pulley & Flipped pulley Preset location
     */
    flipped_pulley flipped_pulley_, flipped_pulley_preset;
    /**
     * @brief AGV1 Preset Locations  
     */
    agv1 agv1_, agv1_drop, agv1_right_;
    /**
     * @brief AGV2 Preset Locations 
     */
    agv2 agv2_, agv2_drop, agv2_right_;
    /**
     * @brief Conveyor up Preset location
     */
    conveyor_up conveyor_up_;
    /**
     * @brief AGV information
     */
    struct agvInfo agv1_allParts, agv2_allParts;
    /**
     * @brief AGV1 allocated flag 
     */
    bool agv1_allocated = false;
    /**
     * @brief AGV2 allocated flag
     */
    bool agv2_allocated = false;
    /**
     * @brief Num preLoc 
     */
    static const int num_preLoc = 20;

  private:
    /**
     * @brief Joint_group_positions_ 
     */
    std::vector<double> joint_group_positions_;
    /**
     * @brief ROS NodeHandle 
     */
    ros::NodeHandle node_;
    /**
     * @brief Planning group 
     */
    std::string planning_group_;
    /**
     * @brief Full robot options 
     */
    moveit::planning_interface::MoveGroupInterface::Options full_robot_options_;
    /**
     * @brief Left arm options 
     */
    moveit::planning_interface::MoveGroupInterface::Options left_arm_options_;
    /**
     * @brief Right arm options 
     */
    moveit::planning_interface::MoveGroupInterface::Options right_arm_options_;
    /**
     * @brief Left ee link options  
     */
    moveit::planning_interface::MoveGroupInterface::Options left_ee_link_options_;
    /**
     * @brief Right ee link options 
     */
    moveit::planning_interface::MoveGroupInterface::Options right_ee_link_options_;
    /**
     * @brief Full robot group 
     */
    moveit::planning_interface::MoveGroupInterface full_robot_group_;
    /**
     * @brief Left arm group 
     */
    moveit::planning_interface::MoveGroupInterface left_arm_group_;
    /**
     * @brief Right arm group 
     */
    moveit::planning_interface::MoveGroupInterface right_arm_group_;
    /**
     * @brief Left ee link group 
     */
    moveit::planning_interface::MoveGroupInterface left_ee_link_group_;
    /**
     * @brief Right ee link group 
     */
    moveit::planning_interface::MoveGroupInterface right_ee_link_group_;
    /**
     * @brief Variable to store if current part is faulty on AGV1
     */
    bool is_part_faulty_agv1 = false;
    /**
     * @brief Variable to store if current part is faulty on AGV2 
     */
    bool is_part_faulty_agv2 = false;
    /**
     * @brief Variable to hold faulty part pose for AGV1 
     */
    geometry_msgs::Pose faulty_part_pose_agv1, part_placed_pose_agv1;
    /**
     * @brief Variable to hold faulty part pose for AGV2 
     */
    geometry_msgs::Pose faulty_part_pose_agv2, part_placed_pose_agv2;
    /**
     * @brief Left ee roll 
     */
    double left_ee_roll_;
    /**
     * @brief Left ee pitch 
     */
    double left_ee_pitch_;
    /**
     * @brief Left ee yaw 
     */
    double left_ee_yaw_;
    /**
     * @brief Left ee quaternion 
     */
    std::array<float,4> left_ee_quaternion_;
    /**
     * @brief Current joint states 
     */
    sensor_msgs::JointState current_joint_states_;
    /**
     * @brief Current left gripper state  
     */
    nist_gear::VacuumGripperState current_left_gripper_state_;
    /**
     * @brief Current right gripper state 
     */
    nist_gear::VacuumGripperState current_right_gripper_state_;
    /**
     * @brief Current gantry controller state 
     */
    control_msgs::JointTrajectoryControllerState current_gantry_controller_state_;
    /**
     * @brief Current left arm controller state  
     */
    control_msgs::JointTrajectoryControllerState current_left_arm_controller_state_;
    /**
     * @brief Current right arm controller state  
     */
    control_msgs::JointTrajectoryControllerState current_right_arm_controller_state_;
    /**
     * @brief Gantry joint trajectory publisher 
     */
    ros::Publisher gantry_joint_trajectory_publisher_;
    /**
     * @brief Left arm joint trajectory publisher 
     */
    ros::Publisher left_arm_joint_trajectory_publisher_;
    /**
     * @brief Right arm joint trajectory publisher 
     */
    ros::Publisher right_arm_joint_trajectory_publisher_;
    /**
     * @brief Joint states subscriber 
     */
    ros::Subscriber joint_states_subscriber_;
    /**
     * @brief Left gripper state subscriber 
     */
    ros::Subscriber left_gripper_state_subscriber_;
    /**
     * @brief Right gripper state subscriber 
     */
    ros::Subscriber right_gripper_state_subscriber_;
    /**
     * @brief Gantry controller state subscriber 
     */
    ros::Subscriber gantry_controller_state_subscriber_;
    /**
     * @brief Left arm controller state subscriber  
     */
    ros::Subscriber left_arm_controller_state_subscriber_;
    /**
     * @brief Right arm controller state subscriber  
     */
    ros::Subscriber right_arm_controller_state_subscriber_;
    /**
     * @brief Left gripper control client 
     */
    ros::ServiceClient left_gripper_control_client;
    /**
     * @brief Right gripper control client 
     */
    ros::ServiceClient right_gripper_control_client;

    // ---------- Callbacks ----------
    /**
     *  @brief Joint states callback
     *  @param joint_state_msg Joint state message
     *  @return None
     */
    void joint_states_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg);
    /**
     *  @brief Left gripper state callback
     *  @param msg Vacuum Gripper State
     *  @return None
     */
    void left_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & msg);
    /**
     *  @brief Right gripper state callback
     *  @param msg Vacuum Gripper State
     *  @return None
     */
    void right_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & msg);
    /**
     *  @brief Gantry controller state callback
     *  @param msg Joint Trajectory Controller State
     *  @return None
     */
    void gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);
    /**
     *  @brief Left arm controller state callback
     *  @param msg Joint Trajectory Controller State
     *  @return None
     */
    void left_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);
    /**
     *  @brief Right arm controller state callback
     *  @param msg JointTrajectoryControllerState
     *  @return None
     */
    void right_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);
    /**
     *  @brief Init stats
     */
    stats init_;
    /**
     *  @brief Move J stats
     */
    stats moveJ_;
    /**
     *  @brief IK Stats
     */
    stats IK_;
    /**
     *  @brief MoveGantry Stats
     */
    stats moveGantry_;
    /**
     *  @brief PickPart Stats
     */
    stats pickPart_;
    /**
     *  @brief PlacePart Stats
     */
    stats placePart_;
    /**
     *  @brief DropPart Stats
     */
    stats dropPart_;
    /**
     *  @brief Grip firmly Stats
     */
    stats gripFirmly_;
    /**
     *  @brief Grip from belt Stats
     */
    stats gripFromBelt_;
    /**
     *  @brief Grip stats
     */
    stats grip_;
};

#endif
