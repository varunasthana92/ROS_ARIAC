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
#include "conveyer.h"
#include "obstacles.h"
#include <nist_gear/LogicalCameraImage.h>


class GantryControl {

  public:
    GantryControl(ros::NodeHandle & node);

    void init();
    stats getStats(std::string function);
    void qualityCallback1(const nist_gear::LogicalCameraImage& msg);
    void qualityCallback2(const nist_gear::LogicalCameraImage& msg);


//    bool moveGantry(std::string waypoints);

//    bool pickPart(part part, std::string arm_name);
    bool pickPart(part part);
    bool placePart(product &product, std::string agv, std::string arm);
    
    /// Send command message to robot controller
    bool send_command(trajectory_msgs::JointTrajectory command_msg);
    void goToPresetLocation(PresetLocation location);
    void rotate_gantry(double angle);
    void activateGripper(std::string gripper_id);
    void deactivateGripper(std::string gripper_id);
    void logicalCallback16(const nist_gear::LogicalCameraImage& msg);
    void logicalCallback17(const nist_gear::LogicalCameraImage& msg);
    void pickFromConveyor(Product &product, ConveyerParts &conveyerPartsObj);
    bool poseMatches(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2);
    bool check_exist_on_agv(const std::string &name, const geometry_msgs::Pose &part_pose, agvInfo &agv);
    void flipPart();
    bool move2start ( float x, float y, float gantryX);
    float move2trg ( float x, float y, float gantryX);
    bool move2closestGap(struct Part &part, std::vector< std::pair<float , float> > &shelfGaps,
                        const std::vector<int> &gapNum, bool actPart, float &gantryX, float &gantryY,
                        ObstaclesInAisle &obj, int &newGap);
//    bool conveyor();

    geometry_msgs::Pose getRobotPose(){
      return full_robot_group_.getCurrentPose().pose;

    }

    nist_gear::VacuumGripperState getGripperState(std::string arm_name);
    geometry_msgs::Pose getTargetWorldPose(geometry_msgs::Pose target, std::string agv, std::string arm);
    //--preset locations;
    start start_;
    bin3 bin3_;
    flipped_pulley flipped_pulley_, flipped_pulley_preset;
    agv1 agv1_, agv1_drop, agv1_right_;
    agv2 agv2_, agv2_drop, agv2_right_;
    conveyor_up conveyor_up_;
    struct agvInfo agv1_allParts, agv2_allParts;
 
    static const int num_preLoc = 20;

  private:
    std::vector<double> joint_group_positions_;
    ros::NodeHandle node_;
    std::string planning_group_;
    moveit::planning_interface::MoveGroupInterface::Options full_robot_options_;
    moveit::planning_interface::MoveGroupInterface::Options left_arm_options_;
    moveit::planning_interface::MoveGroupInterface::Options right_arm_options_;
    moveit::planning_interface::MoveGroupInterface::Options left_ee_link_options_;
    moveit::planning_interface::MoveGroupInterface::Options right_ee_link_options_;
    moveit::planning_interface::MoveGroupInterface full_robot_group_;
    moveit::planning_interface::MoveGroupInterface left_arm_group_;
    moveit::planning_interface::MoveGroupInterface right_arm_group_;
    moveit::planning_interface::MoveGroupInterface left_ee_link_group_;
    moveit::planning_interface::MoveGroupInterface right_ee_link_group_;

    // Variable to store if current part is faulty
    bool is_part_faulty_agv1 = false;
    bool is_part_faulty_agv2 = false;
    // Variable to hold faulty part pose
    geometry_msgs::Pose faulty_part_pose_agv1, part_placed_pose_agv1;
    geometry_msgs::Pose faulty_part_pose_agv2, part_placed_pose_agv2;

    double left_ee_roll_;
    double left_ee_pitch_;
    double left_ee_yaw_;
    std::array<float,4> left_ee_quaternion_;

    sensor_msgs::JointState current_joint_states_;


    nist_gear::VacuumGripperState current_left_gripper_state_;
    nist_gear::VacuumGripperState current_right_gripper_state_;

    control_msgs::JointTrajectoryControllerState current_gantry_controller_state_;
    control_msgs::JointTrajectoryControllerState current_left_arm_controller_state_;
    control_msgs::JointTrajectoryControllerState current_right_arm_controller_state_;

    ros::Publisher gantry_joint_trajectory_publisher_;
    ros::Publisher left_arm_joint_trajectory_publisher_;
    ros::Publisher right_arm_joint_trajectory_publisher_;

    ros::Subscriber joint_states_subscriber_;
    ros::Subscriber left_gripper_state_subscriber_;
    ros::Subscriber right_gripper_state_subscriber_;
    ros::Subscriber gantry_controller_state_subscriber_;
    ros::Subscriber left_arm_controller_state_subscriber_;
    ros::Subscriber right_arm_controller_state_subscriber_;

    ros::ServiceClient left_gripper_control_client;
    ros::ServiceClient right_gripper_control_client;

    // ---------- Callbacks ----------
    void joint_states_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg);
    void left_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & msg);
    void right_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & msg);
    void gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);
    void left_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);
    void right_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);




     // collect stats
    stats init_;
    stats moveJ_;
    stats IK_;
    stats moveGantry_;
    stats pickPart_;
    stats placePart_;
    stats dropPart_;
    stats gripFirmly_;
    stats gripFromBelt_;
    stats grip_;
};

#endif
