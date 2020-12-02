/**
 *  Copyright 2020 Nalin Das
 *  @file gantry_control.cpp
 * 
 *  @author Varun Asthana
 *  @author Saumil Shah
 *  @author Nalin Das
 *  @author Markose Jacob
 *  @author Aditya Goswami
 * 
 *  @date 11/28/2020
 * 
 *  @brief Implements the GantryControl class methods
 * 
 *  @section DESCRIPTION
 *
 *  Source file for the GantryControl class methods
 *
 */
#include "gantry_control.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nist_gear/LogicalCameraImage.h>
#include "utils.h"
#include "orderBuild.h"
#include "conveyer.h"
#include "obstacles.h"


// Quality control sensor 1 callback
void GantryControl::qualityCallback2(const nist_gear::LogicalCameraImage& msg) {
    quality_call_count = (quality_call_count+1)%10000;
    for(auto curr_model : msg.models){
        // ROS_INFO_STREAM("Detected faulty part on agv2 : " << curr_model.type);
        is_part_faulty_agv2 = true;
        geometry_msgs::Pose model_pose = curr_model.pose;
        geometry_msgs::TransformStamped transformStamped;

        bool found=false;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Duration timeout(5.0);
        while(!found) {
            try {
                transformStamped = tfBuffer.lookupTransform("world", "quality_control_sensor_1_frame", ros::Time(0), timeout);
            }
            catch (tf2::TransformException &ex) {
                ROS_FATAL_STREAM( "Not able to find the agv2 quality camera frame -- " << ex.what());
            }
            if(transformStamped.child_frame_id.size()>0) found=true;
        }
        geometry_msgs::Pose world_pose;
        tf2::doTransform(model_pose, world_pose, transformStamped);
        faulty_part_pose_agv2 = world_pose;       
    }
}

void GantryControl::qualityCallback1(const nist_gear::LogicalCameraImage& msg) {
    for(auto curr_model : msg.models){
        // ROS_INFO_STREAM("Detected faulty part on agv1 : " << curr_model.type);
        is_part_faulty_agv1 = true;
        geometry_msgs::Pose model_pose = curr_model.pose;
        geometry_msgs::TransformStamped transformStamped;

        bool found=false;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Duration timeout(5.0);
        while(!found) {
            try {
                transformStamped = tfBuffer.lookupTransform("world", "quality_control_sensor_2_frame", ros::Time(0), timeout);
            }
            catch (tf2::TransformException &ex) {
                ROS_FATAL_STREAM( "Not able to find the agv1 quality camera frame -- " << ex.what());
            }
            if(transformStamped.child_frame_id.size()>0) found=true;
        }
        geometry_msgs::Pose world_pose;
        tf2::doTransform(model_pose, world_pose, transformStamped);
        faulty_part_pose_agv1 = world_pose;       
    }
}

void GantryControl::logicalCallback16(const nist_gear::LogicalCameraImage& msg) {
    for(auto curr_model : msg.models){
        // ROS_INFO_STREAM("Detected part from logical camera 16 on agv1: " << (msg.models[0]).type);
        geometry_msgs::Pose model_pose = curr_model.pose;
        std::string model_name = curr_model.type;

        geometry_msgs::TransformStamped transformStamped;
        bool found=false;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Duration timeout(5.0);
        while(!found) {
            try {
                transformStamped = tfBuffer.lookupTransform("world", "logical_camera_16_frame", ros::Time(0), timeout);
            }
            catch (tf2::TransformException &ex) {
                ROS_FATAL_STREAM( "Not able to find the agv1 camera frame -- " << ex.what());
            }
            if(transformStamped.child_frame_id.size()>0) found=true;
        }
        geometry_msgs::Pose world_pose;
        tf2::doTransform(model_pose, world_pose, transformStamped);


        if(world_pose.position.z < 0.89 && not check_exist_on_agv(model_name, world_pose, agv1_allParts)){
            // ROS_WARN_STREAM("New apart on agv1: " << model_name << " x = " << world_pose.position.x << " y = " << world_pose.position.y);
            part_placed_pose_agv1 = world_pose;
            return;
        }
    }
}

void GantryControl::logicalCallback17(const nist_gear::LogicalCameraImage& msg) {
//    ROS_INFO_STREAM("Detected part from logical camera: " << msg.models.size());
    for(auto curr_model : msg.models){
//         ROS_INFO_STREAM("Detected part from logical camera: " << (msg.models[0]).type);

        geometry_msgs::Pose model_pose = curr_model.pose;
        std::string model_name = curr_model.type;

        geometry_msgs::TransformStamped transformStamped;
        bool found=false;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Duration timeout(5.0);
        while(!found) {
            try {
                transformStamped = tfBuffer.lookupTransform("world", "logical_camera_17_frame", ros::Time(0), timeout);
            }
            catch (tf2::TransformException &ex) {
                ROS_FATAL_STREAM( "Not able to find the agv2 camera frame -- " << ex.what());
            }
            if(transformStamped.child_frame_id.size()>0) found=true;
        }
        geometry_msgs::Pose world_pose;
        tf2::doTransform(model_pose, world_pose, transformStamped);
//        ROS_INFO_STREAM("Current Part name detected by camera: " << model_name );

        if(world_pose.position.z < 0.89 && not check_exist_on_agv(model_name, world_pose, agv2_allParts)){
            // ROS_WARN_STREAM("New part on agv2: " << model_name << " x = " << world_pose.position.x << " y = " << world_pose.position.y);
            part_placed_pose_agv2 = world_pose;
            return;
        }
    }
}

float getDis(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2){

    if(std::abs(p1.position.z - p2.position.z) > 0.09){
        return -1;
    }
    float dis = std::pow((p1.position.x - p2.position.x),2) + std::pow((p1.position.y - p2.position.y),2);
    return std::sqrt(dis);
}

bool GantryControl::check_exist_on_agv(const std::string &name, const geometry_msgs::Pose &part_pose, agvInfo &agv){
    if(agv.prod_on_tray.find(name) != agv.prod_on_tray.end()){
        for(Product &prod: agv.prod_on_tray[name]){
            float dis = getDis(part_pose, prod.agv_world_pose);
            if(dis < 0){
                return false;
            }
            if(dis < 0.03){
                prod.agv_world_pose = part_pose;
                return true;
            }
        }
    }
    return false;
}


bool GantryControl::poseMatches(const geometry_msgs::Pose &pose1, 
                                const geometry_msgs::Pose &pose2) {
    ROS_INFO_STREAM("Trg Pose: (x,y) " << pose1.position.x << " , " << pose1.position.y);

    ROS_INFO_STREAM("Actual Pose: (x,y) " << pose2.position.x << " , " << pose2.position.y );


    // std::vector<double> pose1_angles = quaternionToEuler(pose1);
    // std::vector<double> pose2_angles = quaternionToEuler(pose2);

    // ROS_INFO_STREAM(" Trg Yaw values:" << pose1_angles.at(2));
    // ROS_INFO_STREAM(" Actual Yaw values:" << pose2_angles.at(2));

    ROS_INFO_STREAM("Position Diff (x,y) " << std::abs(pose1.position.x - pose2.position.x) << " , "
                              << std::abs(pose1.position.y - pose2.position.y));

    if (std::abs(pose1.position.x - pose2.position.x)   < 0.03  &&
        std::abs(pose1.position.y - pose2.position.y)   < 0.03
        // std::abs(pose1_angles[2] - pose2_angles[2])     < 7
        ){
        ROS_INFO_STREAM(" Pose Match Status = True");
        return true;
    }
    ROS_INFO_STREAM(" Pose Match Status = False");
    return false;
}

GantryControl::GantryControl(ros::NodeHandle & node):
        node_("/ariac/gantry"),
        planning_group_ ("/ariac/gantry/robot_description"),
        full_robot_options_("Full_Robot",planning_group_,node_),
        left_arm_options_("Left_Arm",planning_group_,node_),
        right_arm_options_("Right_Arm",planning_group_,node_),
        left_ee_link_options_("Left_Endeffector",planning_group_,node_),
        right_ee_link_options_("Right_Endeffector",planning_group_,node_),
        full_robot_group_(full_robot_options_),
        left_arm_group_(left_arm_options_),
        right_arm_group_(right_arm_options_),
        left_ee_link_group_(left_ee_link_options_),
        right_ee_link_group_(right_ee_link_options_)
{
    ROS_INFO_STREAM("[GantryControl::GantryControl] constructor called... ");
}

void GantryControl::init() {
    ROS_INFO_STREAM("[GantryControl::init] init... ");
    double time_called = ros::Time::now().toSec();


    ROS_INFO_NAMED("init", "Planning frame: %s", left_arm_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", right_arm_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", full_robot_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", right_ee_link_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", left_ee_link_group_.getPlanningFrame().c_str());

    ROS_INFO_NAMED("init", "End effector link: %s", left_arm_group_.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("init", "End effector link: %s", right_arm_group_.getEndEffectorLink().c_str());

    left_arm_group_.setPoseReferenceFrame("world");

    //--start location
    start_.gantry = {0,0,0};
    start_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    start_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    bin3_.gantry = {4.0, -1.1, 0.};
    bin3_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    bin3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv1_.gantry = {-0.6, -6.9, PI/4};
    agv1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv1_drop.gantry = {0, -5, 0.};
    agv1_drop.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv1_drop.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv1_right_.gantry = {0.6, -6.9,-PI/2- PI/4};
    agv1_right_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv1_right_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv2_.gantry = {0.6, 6.9, PI + PI/4};
    agv2_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv2_drop.gantry = {0, 5, PI};
    agv2_drop.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv2_drop.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv2_right_.gantry = {-0.6, 6.9, PI/4};
    agv2_right_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv2_right_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //  [right_shoulder_pan_joint, right_shoulder_lift_joint, right_elbow_joint, 
    //  right_wrist_1_joint,       right_wrist_2_joint,       right_wrist_3_joint]
    flipped_pulley_.gantry = {0, 0, 0};
    flipped_pulley_.left_arm = {-1.63, -0.25, 1.61, 6.28, 1.54, 0};
    flipped_pulley_.right_arm = {1.61, -3.20, -1.26, -3.59, -1.57, 0};

    flipped_pulley_preset.gantry = {0, 0, 0};
    flipped_pulley_preset.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, 0, 0, 0};
    flipped_pulley_preset.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    conveyor_up_.gantry = {0, 1.2, 1.57};
    conveyor_up_.left_arm = {0.0, -0.98, 1.95, -0.98, PI/2, 0};
    conveyor_up_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};


    // setPrelocations();
    //--Raw pointers are frequently used to refer to the planning group for improved performance.
    //--To start, we will create a pointer that references the current robot’s state.
    const moveit::core::JointModelGroup* joint_model_group =
            full_robot_group_.getCurrentState()->getJointModelGroup("Full_Robot");

    //--Let’s set a joint space goal and move towards it.
    moveit::core::RobotStatePtr current_state = full_robot_group_.getCurrentState();

    //--Next get the current set of joint values for the group.
//    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);



    gantry_joint_trajectory_publisher_ =
            node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/gantry_controller/command", 10);

    left_arm_joint_trajectory_publisher_ =
            node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/left_arm_controller/command", 10);

    right_arm_joint_trajectory_publisher_ =
            node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/right_arm_controller/command", 10);

    joint_states_subscriber_ = node_.subscribe(
            "/ariac/gantry/joint_states", 10, &GantryControl::joint_states_callback, this);

    left_gripper_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/left_arm/gripper/state", 10, &GantryControl::left_gripper_state_callback, this);

    right_gripper_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/right_arm/gripper/state", 10, &GantryControl::right_gripper_state_callback, this);

    gantry_controller_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/gantry_controller/state", 10, &GantryControl::gantry_controller_state_callback, this);

    left_arm_controller_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/left_arm_controller/state", 10, &GantryControl::left_arm_controller_state_callback, this);

    right_arm_controller_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/right_arm_controller/state", 10, &GantryControl::right_arm_controller_state_callback, this);


    while( (current_gantry_controller_state_.joint_names.size() == 0)
           || (current_left_arm_controller_state_.joint_names.size() == 0)
           || (current_right_arm_controller_state_.joint_names.size() == 0) ) {
        ROS_WARN("[GantryControl::init] Waiting for first controller_state callbacks...");
        ros::Duration(0.1).sleep();
    }

    left_gripper_control_client =
            node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/left_arm/gripper/control");
    left_gripper_control_client.waitForExistence();

    right_gripper_control_client =
            node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/right_arm/gripper/control");
    right_gripper_control_client.waitForExistence();

    // Move robot to init position
    ROS_INFO("[GantryControl::init] Init position ready)...");
}

stats GantryControl::getStats(std::string function) {
    if (function == "init") return init_;
    if (function == "moveJ") return moveJ_;
    if (function == "IK") return IK_;
    if (function == "moveGantry") return moveGantry_;
    if (function == "pickPart") return pickPart_;
    if (function == "placePart") return placePart_;
    if (function == "dropPart") return dropPart_;
    if (function == "gripFirmly") return gripFirmly_;
    if (function == "gripFromBelt") return gripFromBelt_;
    if (function == "grip") return grip_;
}

geometry_msgs::Pose GantryControl::getTargetWorldPose(geometry_msgs::Pose target,
                                                      std::string agv, std::string arm){
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    std::string kit_tray;
    if (agv.compare("agv1")==0)
        kit_tray = "kit_tray_1";
    else
        kit_tray = "kit_tray_2";
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = kit_tray;
    transformStamped.child_frame_id = "target_frame";
    transformStamped.transform.translation.x = target.position.x;
    transformStamped.transform.translation.y = target.position.y;
    transformStamped.transform.translation.z = target.position.z;
    transformStamped.transform.rotation.x = target.orientation.x;
    transformStamped.transform.rotation.y = target.orientation.y;
    transformStamped.transform.rotation.z = target.orientation.z;
    transformStamped.transform.rotation.w = target.orientation.w;


    for (int i{0}; i<15; ++i)
        br.sendTransform(transformStamped);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10);
    ros::Duration timeout(1.0);


    geometry_msgs::TransformStamped world_target_tf;
    geometry_msgs::TransformStamped ee_target_tf;

    std::string ee_link;
    if (arm.compare("left")==0)
        ee_link = "left_ee_link";
    else
        ee_link = "right_ee_link";

    for (int i=0; i< 10; i++) {
        try {
            world_target_tf = tfBuffer.lookupTransform("world", "target_frame",
                                                        ros::Time(0), timeout);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        try {
            ee_target_tf = tfBuffer.lookupTransform("target_frame", ee_link,
                                                 ros::Time(0), timeout);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }

    geometry_msgs::Pose world_target{target};
    // world_target.position.x = world_target_tf.transform.translation.x;
    // world_target.position.y = world_target_tf.transform.translation.y;
    // world_target.position.z = world_target_tf.transform.translation.z;
    // world_target.orientation.x = ee_target_tf.transform.rotation.x;
    // world_target.orientation.y = ee_target_tf.transform.rotation.y;
    // world_target.orientation.z = ee_target_tf.transform.rotation.z;
    // world_target.orientation.w = ee_target_tf.transform.rotation.w;

    world_target.position.x = world_target_tf.transform.translation.x;
    world_target.position.y = world_target_tf.transform.translation.y;
    world_target.position.z = world_target_tf.transform.translation.z;
    world_target.orientation.x = world_target_tf.transform.rotation.x;
    world_target.orientation.y = world_target_tf.transform.rotation.y;
    world_target.orientation.z = world_target_tf.transform.rotation.z;
    world_target.orientation.w = world_target_tf.transform.rotation.w;

    return world_target;
}

void GantryControl::flipPart() {
    goToPresetLocation(flipped_pulley_preset);
    goToPresetLocation(flipped_pulley_);
    activateGripper("right_arm");
    auto state = getGripperState("right_arm");
    while(!state.attached){
        state = getGripperState("right_arm");
        ROS_DEBUG_STREAM_THROTTLE(1,"Tring to activate right gripper " << state.attached << " " << state.enabled);
    }
    deactivateGripper("left_arm");
    // goToPresetLocation(flipped_pulley_preset);
    goToPresetLocation(start_);
    return;
}

bool GantryControl::pickPart(part part){
    //--Activate gripper
    activateGripper("left_arm");
    geometry_msgs::Pose currentPose = left_arm_group_.getCurrentPose().pose;

    if(part.type.size()==0) {
        ROS_INFO_STREAM("Part name is not present");
        return false;
    }
    ROS_INFO_STREAM("Part Z in Pick "<< part.pose.position.z);
    part.pose.position.z = part.pose.position.z + model_height[part.type] + GRIPPER_HEIGHT - EPSILON;
    part.pose.orientation.x = currentPose.orientation.x;
    part.pose.orientation.y = currentPose.orientation.y;
    part.pose.orientation.z = currentPose.orientation.z;
    part.pose.orientation.w = currentPose.orientation.w;



    //    ROS_INFO_STREAM("["<< part.type<<"]= " << part.pose.position.x << ", " << part.pose.position.y << "," << part.pose.position.z << "," << part.pose.orientation.x << "," << part.pose.orientation.y << "," << part.pose.orientation.z << "," << part.pose.orientation.w);
    bool *is_part_faulty;
    geometry_msgs::Pose* faulty_part_pose;
    if(part.agv_id == "agv1"){
    	is_part_faulty = &is_part_faulty_agv1;
    	faulty_part_pose = &faulty_part_pose_agv1;
    }else{
    	is_part_faulty = &is_part_faulty_agv2;
    	faulty_part_pose = &faulty_part_pose_agv2;
    }
	auto state = getGripperState("left_arm");
    while(!state.enabled){
        activateGripper("left_arm");
        state = getGripperState("left_arm");
    }
    
    if (state.enabled) {
        ROS_INFO_STREAM("[Gripper] = enabled");
        //--Move arm to part
        if(*is_part_faulty){
            part.pose.position.z = 0.73 + model_height[part.type];
            // part.pose.position.z = part.pose.position.z + model_height[part.type] + GRIPPER_HEIGHT - EPSILON;
            ROS_WARN_STREAM("Trying faulty pick");
        }
        if(part.obstacle_free){
            part.pose.position.z += 0.2;
            left_arm_group_.setPoseTarget(part.pose);
            left_arm_group_.move();
            part.pose.position.z -= 0.2;
        }
        

        ROS_INFO_STREAM("Z in Pick part "<< part.pose.position.z);
        left_arm_group_.setPoseTarget(part.pose);
        left_arm_group_.move();
        auto state = getGripperState("left_arm");
        for(int i = 0; i < 50; ++ i){
            activateGripper("left_arm");
            state = getGripperState("left_arm");
        }
        if (state.attached) {
            ROS_INFO_STREAM("[Gripper] = object attached");
            //--Move arm to previous position
            if (part.obstacle_free == false){
                part.pose.position.y = currentPose.position.y;
            }
            part.pose.position.z += 0.2;
            left_arm_group_.setPoseTarget(part.pose);
            left_arm_group_.move();
            return true;
        }
        else {
            ROS_INFO_STREAM("[Gripper] = object not attached");
            if(part.obstacle_free == false){
                return false;
            }
            if (*is_part_faulty){
                part.pose = *faulty_part_pose;
                part.pose.position.z = 0.73 + model_height[part.type];
                // part.pose.position.z = part.pose.position.z + model_height[part.type] + GRIPPER_HEIGHT - EPSILON;
                ROS_INFO_STREAM("Z in Pick faulty trial "<< part.pose.position.z);
                // part.pose.position.z = part.pose.position.z + model_height.at(part.type) + GRIPPER_HEIGHT - EPSILON - 0.0516;
                part.pose.orientation.x = currentPose.orientation.x;
                part.pose.orientation.y = currentPose.orientation.y;
                part.pose.orientation.z = currentPose.orientation.z;
                part.pose.orientation.w = currentPose.orientation.w;
            }

            int max_attempts{4};
            int current_attempt{0};
            // while(!state.attached && current_attempt <= max_attempts) {
            while(!state.attached) {
                activateGripper("left_arm");
                // part.pose.position.y = currentPose.position.y;
                part.pose.position.z += 0.2;
                left_arm_group_.setPoseTarget(part.pose);
                left_arm_group_.move();
                part.pose.position.z -= 0.2;

                left_arm_group_.setPoseTarget(part.pose);
                left_arm_group_.move();
                state = getGripperState("left_arm");

                current_attempt++;
            }

            // if(!state.attached){
            //     return false;
            // }
        }
        return true;
    }
    else {
        ROS_INFO_STREAM("[Gripper] = not enabled");
    }
    return false;
}

bool GantryControl::placePart(Product &product, std::string agv,
                              std::string arm, struct all_Order *curr_prod, ConveyerParts &conveyerPartsObj){

    Part part = product.p;
    auto target_pose_in_tray = getTargetWorldPose(product.pose, agv, arm);
    // target_pose_in_tray += model_height[part.type];
    product.rpy_final = quaternionToEuler(target_pose_in_tray);
    
    ROS_INFO_STREAM("Settled tray World pose:" << target_pose_in_tray.position.x << " " 
                                            << target_pose_in_tray.position.y << " "
                                            << target_pose_in_tray.position.z);
//    ros::Duration(3.0).sleep();
    auto left_state = getGripperState("left_arm");
    auto right_state = getGripperState("right_arm");
    float target_z_org = target_pose_in_tray.position.z + model_height[part.type];
    target_pose_in_tray.position.z += (ABOVE_TARGET + 1.5*model_height[part.type]);

    // target_pose_in_tray.position.z = target_pose_in_tray.position.z + model_height.at(part.type) + GRIPPER_HEIGHT + 0.08;

    geometry_msgs::Pose currentPose;
    PresetLocation agv_in_use;
    PresetLocation agv_in_use_drop;
    PresetLocation agv_in_use_right;
    struct agvInfo* agv_data;

    float offset_x = 0.2;
    float offset_y = 0.6;

    if(agv == "agv1"){
        agv_in_use = agv1_;
        agv_in_use_drop = agv1_drop;
        agv_in_use_right = agv1_right_;
        agv_data = &agv1_allParts;
        offset_x *=-1;
        offset_y *=-1;
    }else{
        agv_in_use = agv2_;
        agv_in_use_drop = agv2_drop;
        agv_in_use_right = agv2_right_;
        agv_data = &agv2_allParts;
    }

    ROS_WARN_STREAM("Product on " << agv);
    for(auto prod_map: agv_data->prod_on_tray){
        std::cout << prod_map.first << " : " << prod_map.second.size() << std::endl;
        std::cout << "With poses - " << std::endl; 
        for(auto allProd : prod_map.second){
            std::cout << allProd.agv_world_pose.position <<std::endl;
        }
    }

    tf2::Quaternion q_pitch( 0, 0.7071068, 0, 0.7071068);
    
    tf2::Quaternion q_pi( 0, 0, 1, 0);

    tf2::Quaternion q_init_part(part.pose.orientation.x,
                                part.pose.orientation.y,
                                part.pose.orientation.z,
                                part.pose.orientation.w);

    tf2::Quaternion q_final_part(target_pose_in_tray.orientation.x,
                                target_pose_in_tray.orientation.y,
                                target_pose_in_tray.orientation.z,
                                target_pose_in_tray.orientation.w);

    tf2::Quaternion q_pi_by_2( 0, 0, 0.7071068, 0.7071068);

    if (left_state.attached) {
        // agv_in_use.gantry[0] = target_pose_in_tray.position.x + offset_x;
        // agv_in_use.gantry[1] = -target_pose_in_tray.position.y - offset_y;

        agv_in_use.gantry[0] = target_pose_in_tray.position.x + offset_x;
        agv_in_use.gantry[1] = -target_pose_in_tray.position.y - offset_y;
        goToPresetLocation(agv_in_use);

        
        tf2::Quaternion q_rslt = q_init_part.inverse()*q_final_part*q_pi*q_pitch;
        target_pose_in_tray.orientation.x = q_rslt.x();
        target_pose_in_tray.orientation.y = q_rslt.y();
        target_pose_in_tray.orientation.z = q_rslt.z();
        target_pose_in_tray.orientation.w = q_rslt.w();

        left_state = getGripperState("left_arm");
        if (left_state.attached){
            currentPose = left_arm_group_.getCurrentPose().pose;
            left_arm_group_.setPoseTarget(target_pose_in_tray);
            left_arm_group_.move();

            deactivateGripper("left_arm");
            // left_arm_group_.setPoseTarget(currentPose);
            // left_arm_group_.move();
        }
    } else if (right_state.attached){

        agv_in_use_right.gantry[0] = target_pose_in_tray.position.x + offset_x;
        agv_in_use_right.gantry[1] = -target_pose_in_tray.position.y - offset_y;
        goToPresetLocation(agv_in_use_right);

        tf2::Quaternion q_rslt = q_init_part.inverse()*q_final_part*q_pitch;
        target_pose_in_tray.orientation.x = q_rslt.x();
        target_pose_in_tray.orientation.y = q_rslt.y();
        target_pose_in_tray.orientation.z = q_rslt.z();
        target_pose_in_tray.orientation.w = q_rslt.w();

        right_state = getGripperState("right_arm");
        if (right_state.attached){
            // currentPose = right_arm_group_.getCurrentPose().pose;
            // target_pose_in_tray.orientation.x = currentPose.orientation.x;
            // target_pose_in_tray.orientation.y = currentPose.orientation.y;
            // target_pose_in_tray.orientation.z = currentPose.orientation.z;
            // target_pose_in_tray.orientation.w = currentPose.orientation.w;

            right_arm_group_.setPoseTarget(target_pose_in_tray);
            right_arm_group_.move();

            deactivateGripper("right_arm");
            // right_arm_group_.setPoseTarget(currentPose);
            // right_arm_group_.move();
        }
    }

    bool *is_part_faulty;
    geometry_msgs::Pose* part_placed_pose;
    geometry_msgs::Pose* faulty_part_pose;

    if(part.agv_id == "agv2"){
    	is_part_faulty = &is_part_faulty_agv2;
    	faulty_part_pose = &faulty_part_pose_agv2;
        part_placed_pose = &part_placed_pose_agv2;
    }else{
    	is_part_faulty = &is_part_faulty_agv1;
    	faulty_part_pose = &faulty_part_pose_agv1;
        part_placed_pose = &part_placed_pose_agv1;
    }

    ros::Duration(1).sleep();
    int temp_call_check = quality_call_count;
    while(temp_call_check == quality_call_count){
        conveyerPartsObj.updateCurrentPoses();
        conveyerPartsObj.checkBoundaries();
        conveyerPartsObj.checkCurrentPartSet();
        ROS_WARN_STREAM_THROTTLE(5, "-#########  BLACK OUT  ##########");
    }

    if (*is_part_faulty) {
        ROS_INFO_STREAM("-----------------Part faulty: " << *is_part_faulty);
        part.pose = *faulty_part_pose;

        if(product.p.flip_part == true){
            agv_in_use_right.gantry[0] = part.pose.position.x + offset_x;
            agv_in_use_right.gantry[1] = -part.pose.position.y - (2*offset_y);
            goToPresetLocation(agv_in_use_right);
        }
        
        agv_in_use.gantry[0] = part.pose.position.x + offset_x;
        agv_in_use.gantry[1] = -part.pose.position.y - offset_y;
        goToPresetLocation(agv_in_use);
        pickPart(part);
        goToPresetLocation(agv_in_use); // to avoid part drag on tray, need to pull the parm
        goToPresetLocation(agv_in_use_drop);
        deactivateGripper("left_arm");
        *is_part_faulty = false;
        return false;
    }

    // ros::Duration(1).sleep();
    ROS_INFO_STREAM("-----------------Matching Pose for : " << part.type << " agv: " << part.agv_id);
    // ros::Duration(3).sleep();
    bool is_part_placed_correct = poseMatches(target_pose_in_tray, *part_placed_pose);

    // bool temp_flip_status = product.p.flip_part;
    if(product.p.flip_part == true && product.p.flip_part_correction == false){
        ROS_INFO_STREAM("-----------------Correcting pose for flipped part");
        is_part_placed_correct = false;
        product.p.flip_part_correction = true;
    }

    if(is_part_placed_correct){
        product.agv_world_pose = *part_placed_pose;
        product.p.pose = *part_placed_pose;
        product.part_placed = true;
        agv_data->prod_on_tray[product.type].push_back(product);
        agv_data->complete_order_data.push_back(curr_prod);
        agv_data->count++;
    }else{        
        geometry_msgs::Pose original_part_pose = part.pose;
        part.pose = *part_placed_pose;
        if(product.p.flip_part == true && product.p.flip_part_preset_correction == false){
            agv_in_use_right.gantry[0] = target_pose_in_tray.position.x + offset_x;
            agv_in_use_right.gantry[1] = -target_pose_in_tray.position.y - (2*offset_y);
            goToPresetLocation(agv_in_use_right);
            product.p.flip_part_preset_correction = true;
        }

        agv_in_use.gantry[0] = part.pose.position.x + offset_x;
        agv_in_use.gantry[1] = -part.pose.position.y - offset_y;
        goToPresetLocation(agv_in_use);
        tf2::Quaternion q_init_part_temp(part.pose.orientation.x,
                                        part.pose.orientation.y,
                                        part.pose.orientation.z,
                                        part.pose.orientation.w);
        
        currentPose = left_arm_group_.getCurrentPose().pose;
        tf2::Quaternion q_robot(currentPose.orientation.x,
                                currentPose.orientation.y,
                                currentPose.orientation.z,
                                currentPose.orientation.w);

        tf2::Quaternion q_pi_by_4( 0, 0, -0.3826834, 0.9238795);

        tf2::Quaternion q_res = q_init_part_temp*q_pi_by_4;

        if(agv == "agv2"){
            q_res = q_res*q_pi;
        }
        if(product.p.flip_part){
            tf2::Quaternion q_flip( 1, 0, 0, 0);
            q_res = q_res*q_pi_by_2*q_flip.inverse();
        }
        part.pose.orientation.x = q_res.x();
        part.pose.orientation.y = q_res.y();
        part.pose.orientation.z = q_res.z();
        part.pose.orientation.w = q_res.w();
        product.p.pose = part.pose;
        *is_part_faulty =  true;
        pickPart(part);
        *is_part_faulty =  false;
        bool placed = placePart(product, agv, arm, curr_prod, conveyerPartsObj);
    }
    // product.p.flip_part = temp_flip_status;
    return true;
}

void GantryControl::clearAgv(std::string agv, BuildClass &buildObj){

    PresetLocation agv_in_use;
    PresetLocation agv_in_use_drop;
    struct agvInfo* agv_data;

    float offset_x = 0.2;
    float offset_y = 0.6;

    if(agv == "agv1"){
        agv_in_use = agv1_;
        agv_in_use_drop = agv1_drop;
        agv_data = &agv1_allParts;
        offset_x *=-1;
        offset_y *=-1;
    }else{
        agv_in_use = agv2_;
        agv_in_use_drop = agv2_drop;
        agv_data = &agv2_allParts;
    }

    ROS_WARN_STREAM("Clearing Product on " << agv);
    for(auto prod_map: agv_data->prod_on_tray){
        std::cout << prod_map.first << " : " << prod_map.second.size() << std::endl;
        std::cout << "With poses - " << std::endl; 
        for(auto allProd : prod_map.second){
            std::cout << allProd.agv_world_pose.position <<std::endl;
            Part remove_part = allProd.p;
            remove_part.pose  = allProd.agv_world_pose;

            agv_in_use.gantry[0] = remove_part.pose.position.x + offset_x;
            agv_in_use.gantry[1] = -remove_part.pose.position.y - offset_y;
            goToPresetLocation(agv_in_use);
            
            auto currentPose = left_arm_group_.getCurrentPose().pose;
            remove_part.pose.orientation = currentPose.orientation;
            remove_part.pose.position.z = 0.73;

            pickPart(remove_part);
            goToPresetLocation(agv_in_use); // to avoid part drag on tray, need to pull the parm
            goToPresetLocation(agv_in_use_drop);
            deactivateGripper("left_arm");
        }
    }

    agv_data->prod_on_tray.clear();
    agv_data->count = 0;
    ROS_WARN_STREAM("clearAGV() complete_order_data.size() : " << agv_data->complete_order_data.size());
    for(auto curr_prod : agv_data->complete_order_data){
        buildObj.pushList(curr_prod);
        buildObj.ship_build_count[curr_prod->ship_num]--;
    }
    agv_data->complete_order_data.clear();
    return;
}

bool GantryControl::move2start ( float x, float y, std::vector<double> left_arm) {
    float offset_y = 0.2;
    float offset_x = 0.2;

    PresetLocation move;
    move.gantry = {x,y,0};
    move.left_arm = left_arm;
    move.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    if(x == 0 && y == 0 ){
        return true;
    }

    if(x < 0){
        if( y > 6.6){
            move.gantry[0] = 0;
            move.gantry[1] -= offset_y;

            ROS_INFO_STREAM("Position of str_1");

            goToPresetLocation(move);
            move.left_arm = start_.left_arm;
            move.right_arm = start_.right_arm;
            goToPresetLocation(move);
            return true;
        }
        else  if( y <= 6.6 && y > 3.05){
            move.gantry[0] = x;
            move.gantry[1] += offset_y;
            // move.left_arm = {-PI/2 , -PI/2, -PI/2 - PI/4 , -PI/2 - PI/4, 0, 0};

            ROS_INFO_STREAM("Position of str_2");

            goToPresetLocation(move);
            move.gantry[0] = 0;
            // move.gantry[1] += offset_y;
            // move.left_arm = {-PI/2 , -PI/2, -PI/2 - PI/4 , -PI/2 - PI/4, 0, 0};
            goToPresetLocation(move);
            move.left_arm = start_.left_arm;
            move.right_arm = start_.right_arm;
            goToPresetLocation(move);
            return true;
        }
        else if( y <= 3.05 && y > 1.5025){
            move.gantry[0] = x;
            move.gantry[1] -= offset_y;
            // move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};

            ROS_INFO_STREAM("Position of str_3");

            goToPresetLocation(move);

            move.gantry[0] = 0;
            // move.gantry[1] -= offset_y;
            // move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};
            goToPresetLocation(move);
            move.left_arm = start_.left_arm;
            move.right_arm = start_.right_arm;
            goToPresetLocation(move);
            return true;
        }
        else if( y <= 1.5025 && y > 0){
            move.gantry[0] = x;
            move.gantry[1] += offset_y;
            // move.left_arm = {-PI/2 , -PI/2, -PI/2 - PI/4 , -PI/2 - PI/4, 0, 0};

            ROS_INFO_STREAM("Position of str_4");

            goToPresetLocation(move);
            move.gantry[0] = 0;
            // move.gantry[1] += offset_y;
            // move.left_arm = {-PI/2 , -PI/2, -PI/2 - PI/4 , -PI/2 - PI/4, 0, 0};
            goToPresetLocation(move);
            move.left_arm = start_.left_arm;
            move.right_arm = start_.right_arm;
            goToPresetLocation(move);
            return true;
        }
        else if( y <= 0 && y > -1.5025){
            move.gantry[0] = x;
            move.gantry[1] -= offset_y;
            // move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};

            ROS_INFO_STREAM("Position of str_5");

            goToPresetLocation(move);

            move.gantry[0] = 0;
            // move.gantry[1] -= offset_y;
            // move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};
            goToPresetLocation(move);
            move.left_arm = start_.left_arm;
            move.right_arm = start_.right_arm;
            goToPresetLocation(move);
            return true;
        }
        else if( y <= -1.5025 && y > -3.05){
            move.gantry[0] = x;
            move.gantry[1] += offset_y;
            // move.left_arm = {-PI/2 , -PI/2, -PI/2 - PI/4 , -PI/2 - PI/4, 0, 0};

            ROS_INFO_STREAM("Position of str_6");

            goToPresetLocation(move);
            move.gantry[0] = 0;
            // move.gantry[1] += offset_y;
            // move.left_arm = {-PI/2 , -PI/2, -PI/2 - PI/4 , -PI/2 - PI/4, 0, 0};
            goToPresetLocation(move);
            move.left_arm = start_.left_arm;
            move.right_arm = start_.right_arm;
            goToPresetLocation(move);
            return true;
        }
        else if( y <= -3.05 && y > -6.6){
            move.gantry[0] = x;
            move.gantry[1] -= offset_y;
            // move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};

            ROS_INFO_STREAM("Position of str_7");

            goToPresetLocation(move);

            move.gantry[0] = 0;
            // move.gantry[1] -= offset_y;
            // move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};
            goToPresetLocation(move);
            move.left_arm = start_.left_arm;
            move.right_arm = start_.right_arm;
            goToPresetLocation(move);
            return true;
        }
        else if( y <= -6.6){
            move.gantry[0] = 0;
            move.gantry[1] += offset_y;

            ROS_INFO_STREAM("Position of str_8");

            goToPresetLocation(move);
            move.left_arm = start_.left_arm;
            move.right_arm = start_.right_arm;
            goToPresetLocation(move);
            return true;
        }
        return false;

    }else{ // towards bin side
        if( y > 6.6){
            move.gantry[0] = 0;
            move.gantry[1] -= offset_y;

            ROS_INFO_STREAM("Position of str_9");

            goToPresetLocation(move);
            move.left_arm = start_.left_arm;
            move.right_arm = start_.right_arm;
            goToPresetLocation(move);
            return true;
        }
        else  if( y <= 6.6 && y > 3.6){

            move.gantry[0] = x;
            move.gantry[1] += offset_y;
            // move.left_arm = {-PI/2 , -PI/2, -PI/2 - PI/4 , -PI/2 - PI/4, 0, 0};

            ROS_INFO_STREAM("Position of str_10");

            goToPresetLocation(move);
            move.gantry[0] = 0;
            // move.gantry[1] += offset_y;
            // move.left_arm = {-PI/2 , -PI/2, -PI/2 - PI/4 , -PI/2 - PI/4, 0, 0};
            goToPresetLocation(move);
            move.left_arm = start_.left_arm;
            move.right_arm = start_.right_arm;
            goToPresetLocation(move);
            return true;
        }
        else  if( y <= 3.6 && y > 2.45){
            move.gantry[0] = x;
            move.gantry[1] -= offset_y;
            ROS_INFO_STREAM("Position of str_11");
            goToPresetLocation(move);

            move.gantry[0] = 0;
            move.left_arm = start_.left_arm;
            move.right_arm = start_.right_arm;
            goToPresetLocation(move);
            return true;
        }
        else  if( y <= 2.45 && y > 0){
            move.gantry[0] = x;
            move.gantry[1] -= offset_y+0.5;

            ROS_INFO_STREAM("Position of str_bins1");

            goToPresetLocation(move);
            move.gantry[0] = 0;
            move.left_arm = start_.left_arm;
            move.right_arm = start_.right_arm;
            goToPresetLocation(move);
            return true;
        }
        else  if( y <= -2.45 && y > -3.6){
            move.gantry[0] = x;
            move.gantry[1] += offset_y;
            // move.left_arm = {-PI/2 , -PI/2, -PI/2 - PI/4 , -PI/2 - PI/4, 0, 0};

            ROS_INFO_STREAM("Position of str_12");

            goToPresetLocation(move);
            move.gantry[0] = 0;
            // move.gantry[1] += offset_y;
            // move.left_arm = {-PI/2 , -PI/2, -PI/2 - PI/4 , -PI/2 - PI/4, 0, 0};
            goToPresetLocation(move);
            move.left_arm = start_.left_arm;
            move.right_arm = start_.right_arm;
            goToPresetLocation(move);
            return true;
        }
        else  if( y <= 0 && y > -2.45){
            move.gantry[0] = x;
            move.gantry[1] += offset_y+0.5;

            ROS_INFO_STREAM("Position of str_bins2");

            goToPresetLocation(move);
            move.gantry[0] = 0;
            move.left_arm = start_.left_arm;
            move.right_arm = start_.right_arm;
            goToPresetLocation(move);
            return true;
        }
        else  if( y <= -3.6 && y > -6.6){
            move.gantry[0] = x;
            move.gantry[1] -= offset_y;
            // move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};

            ROS_INFO_STREAM("Position of str_13");

            goToPresetLocation(move);

            move.gantry[0] = 0;
            // move.gantry[1] -= offset_y;
            // move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};
            goToPresetLocation(move);
            move.left_arm = start_.left_arm;
            move.right_arm = start_.right_arm;
            goToPresetLocation(move);
            return true;
        }
        else  if( y <= -6.6 ){
            move.gantry[0] = 0;
            move.gantry[1] += offset_y;

            ROS_INFO_STREAM("Position of str_14");

            goToPresetLocation(move);
            move.left_arm = start_.left_arm;
            move.right_arm = start_.right_arm;
            goToPresetLocation(move);
            return true;
        }
        return false;
    }
}

std::vector<double> GantryControl::move2trg  ( float x, float y, float &gantryX, float &gantryY , int currGap, std::vector<double> left_arm) {

    float offset_final_y = 1.1;
    float offset_y = offset_final_y + 0.2;
    float tune_even_y_bin_side = 0.2; //controller to fine tune offset in y axis only for right side of the shelf's on bin side
    float tune_odd_y = 0.2; //controller to fine tune offset in y axis only for left side of the shelf's on -ve x side
    float tune_even_y = 0.2; //controller to fine tune offset in y axis only for right side of the shelf's on -ve x side
    float bin_tune_y = 0.6; //controller to fine tune offset in y axis for bins to move toward y = 0
    float bin_tune_x = 0.75; //controller to fine tune offset in x axis for bins to move towards the conveyor
    float offset_final_x = 0.6;  //Earlier value was 0.4. Changed to 0.6 because arm was colliding with shelf while picking up

    PresetLocation move, move_trg;
    move.gantry = {x,y,0};
    // move.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};

    move.left_arm = left_arm;
    move.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    move_trg = move;
    // if(currGap != -1){
    //     move.left_arm = left_arm;
    //     move.right_arm = { PI, 0, 0, 0, 0, 0};
    // }

    if(x < 0){
        if( y > 6.3){
            move.gantry[0] = gantryX;
            move.gantry[1] -= offset_y;
            
            ROS_INFO_STREAM("Position of trg  trg_1");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] -= offset_final_y;
            goToPresetLocation(move_trg);

            gantryX = move_trg.gantry[0];
            gantryY = -move_trg.gantry[1];
            return move_trg.left_arm;
        }
        else  if( y <= 6.3 && y > 3.05){
            offset_y -= tune_even_y;
            move.gantry[0] = gantryX;
            move.gantry[1] += offset_y;


            // move.left_arm = {-PI/2 , -PI/2, -PI/2 - PI/4 , -PI/2 - PI/4, 0, 0};

            ROS_INFO_STREAM("Position of trg  trg_2");

            goToPresetLocation(move);

            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] += offset_final_y  - tune_even_y - 0.2;
            move_trg.left_arm = {-PI/2 , -PI/2, -PI/2 - PI/4 , -PI/2 - PI/4, 0, 0};
            goToPresetLocation(move_trg);
            gantryX = move_trg.gantry[0];
            gantryY = -move_trg.gantry[1];
            return move_trg.left_arm;
        }
        else if( y <= 3.05 && y > 1.5025){
            move.gantry[0] = gantryX;
            move.gantry[1] -= offset_y;
            // move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};

            ROS_INFO_STREAM("Position of trg  trg_3");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] -= offset_final_y - tune_odd_y;
            move_trg.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};
            goToPresetLocation(move_trg);
            gantryX = move_trg.gantry[0];
            gantryY = -move_trg.gantry[1];
            return move_trg.left_arm;

        }
        else if( y <= 1.5025 && y > 0){
            offset_y -= tune_even_y;
            offset_final_y -= tune_even_y;
            move.gantry[0] = gantryX;
            move.gantry[1] += offset_y;
            // move.left_arm = {-PI/2 , -PI/2, -PI/2 - PI/4 , -PI/2 - PI/4, 0, 0};

            ROS_INFO_STREAM("Position of trg  trg_4");

            goToPresetLocation(move);

            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] += offset_final_y;
            move_trg.left_arm = {-PI/2 , -PI/2, -PI/2 - PI/4 , -PI/2 - PI/4, 0, 0};
            goToPresetLocation(move_trg);
            gantryX = move_trg.gantry[0];
            gantryY = -move_trg.gantry[1];
            return move_trg.left_arm;

        }
        else if( y <= 0 && y > -1.5025){
            move.gantry[0] = gantryX;
            move.gantry[1] -= offset_y;
            // move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};

            ROS_INFO_STREAM("Position of trg  trg_5");

            goToPresetLocation(move);

            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] -= offset_final_y - tune_odd_y;
            move_trg.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};
            goToPresetLocation(move_trg);
            gantryX = move_trg.gantry[0];
            gantryY = -move_trg.gantry[1];
            return move_trg.left_arm;
        }
        else if( y <= -1.5025 && y > -3.05){
            offset_y -= tune_even_y;
            offset_final_y -= tune_even_y;
            move.gantry[0] = gantryX;
            move.gantry[1] += offset_y;
            // move.left_arm = {-PI/2 , -PI/2, -PI/2 - PI/4 , -PI/2 - PI/4, 0, 0};

            ROS_INFO_STREAM("Position of trg  trg_6");

            goToPresetLocation(move);

            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] += offset_final_y;
            move_trg.left_arm = {-PI/2 , -PI/2, -PI/2 - PI/4 , -PI/2 - PI/4, 0, 0};
            goToPresetLocation(move_trg);
            gantryX = move_trg.gantry[0];
            gantryY = -move_trg.gantry[1];
            return move_trg.left_arm;
        }
        else if( y <= -3.05 && y > -6.6){
            move.gantry[0] = gantryX;
            move.gantry[1] -= offset_y;
            // move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};

            ROS_INFO_STREAM("Position of trg  trg_7");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] -= offset_final_y - tune_odd_y;
            move_trg.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};
            goToPresetLocation(move_trg);
            gantryX = move_trg.gantry[0];
            gantryY = -move_trg.gantry[1];
            return move_trg.left_arm;
        }
        else if( y <= -6.6){
            move.gantry[0] = gantryX;
            move.gantry[1] += offset_y;

            ROS_INFO_STREAM("Position of trg  trg_8");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] += offset_final_y;
            goToPresetLocation(move_trg);
            gantryX = move_trg.gantry[0];
            gantryY = -move_trg.gantry[1];
            return move_trg.left_arm;
        }
        return move_trg.left_arm;;

    }else{ // towards bin s
        if( y > 6.3){
            move.gantry[0] = gantryX;
            move.gantry[1] -= offset_y;

            ROS_INFO_STREAM("Position of trg  trg_9");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] -= offset_final_y;
            goToPresetLocation(move_trg);
            gantryX = move_trg.gantry[0];
            gantryY = -move_trg.gantry[1];
            return move_trg.left_arm;
        }
        else  if( y <= 6.3 && y > 3.6){
            offset_y -= tune_even_y_bin_side;
            move.gantry[0] = gantryX;
            move.gantry[1] += offset_y;
            // move.left_arm = {-PI/2 , -PI/2, -PI/2 - PI/4 , -PI/2 - PI/4, 0, 0};

            ROS_INFO_STREAM("Position of trg  trg_10");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] += offset_final_y-0.2;
            move_trg.left_arm = {-PI/2 , -PI/2, -PI/2 - PI/4 , -PI/2 - PI/4, 0, 0};
            goToPresetLocation(move_trg);
            gantryX = move_trg.gantry[0];
            gantryY = -move_trg.gantry[1];
            return move_trg.left_arm;
        }
        else  if( y <= 3.6 && y > 2.45) {
            move.gantry[0] = gantryX;
            move.gantry[1] -= offset_y;
            // move.left_arm = {-PI / 2, -PI / 2, PI / 2 + PI / 4, -PI / 4, 0, 0};

            ROS_INFO_STREAM("Position of trg  trg_11");

            goToPresetLocation(move);

            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] -= offset_final_y - tune_odd_y;
            move_trg.left_arm = {-PI / 2, -PI / 2, PI / 2 + PI / 4, -PI / 4, 0, 0};
            goToPresetLocation(move_trg);
            gantryX = move_trg.gantry[0];
            gantryY = -move_trg.gantry[1];
            return move_trg.left_arm;
        }
        else  if( y <= 2.45 && y > 0) {
            offset_y = bin_tune_y;
            offset_final_x = bin_tune_x;
            move.gantry[0] = gantryX;

            ROS_INFO_STREAM("Position of trg  trg_bins1");

            goToPresetLocation(move);

            move_trg.gantry[0] -= offset_final_x;
//            move_trg.gantry[1] -= offset_y;
            goToPresetLocation(move_trg);
            gantryX = move_trg.gantry[0];
            gantryY = -move_trg.gantry[1];
            return move_trg.left_arm;
        }
        else  if( y <= 0 && y > -2.45){
            offset_y = bin_tune_y;
            offset_final_x = bin_tune_x;
            move.gantry[0] = gantryX;

            ROS_INFO_STREAM("Position of trg  trg_bins2");

            goToPresetLocation(move);

            move_trg.gantry[0] -= offset_final_x;
//            move_trg.gantry[1] -= offset_y;
            goToPresetLocation(move_trg);
            gantryX = move_trg.gantry[0];
            gantryY = -move_trg.gantry[1];
            return move_trg.left_arm;
        }

        else  if( y <= -2.45 && y > -3.6){
            offset_y -= tune_even_y_bin_side;
            offset_final_y -= tune_even_y_bin_side;
            move.gantry[0] = gantryX;
            move.gantry[1] += offset_y;
            // move.left_arm = {-PI/2 , -PI/2, -PI/2 - PI/4 , -PI/2 - PI/4, 0, 0};

            ROS_INFO_STREAM("Position of trg  trg_12");

            goToPresetLocation(move);

            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] += offset_final_y;
            move_trg.left_arm = {-PI/2 , -PI/2, -PI/2 - PI/4 , -PI/2 - PI/4, 0, 0};
            goToPresetLocation(move_trg);
            gantryX = move_trg.gantry[0];
            gantryY = -move_trg.gantry[1];
            return move_trg.left_arm;
        }
        
        else  if( y <= -3.6 && y > -6.6){
            move.gantry[0] = gantryX;
            move.gantry[1] -= offset_y;
            // move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};

            ROS_INFO_STREAM("Position of trg  trg_13");

            goToPresetLocation(move);

            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] -= offset_final_y - tune_odd_y +0.2;
            move_trg.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};
            goToPresetLocation(move_trg);
            gantryX = move_trg.gantry[0];
            gantryY = -move_trg.gantry[1];
            return move_trg.left_arm;
        }
        else  if( y <= -6.6 ){
            move.gantry[0] = gantryX;
            move.gantry[1] += offset_y;

            ROS_INFO_STREAM("Position of trg  trg_14");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] += offset_final_y;
            goToPresetLocation(move_trg);
            gantryX = move_trg.gantry[0];
            gantryY = -move_trg.gantry[1];
            return move_trg.left_arm;
        }
        return move_trg.left_arm;
   }
}

bool GantryControl::pickFromConveyor(Product &product, ConveyerParts &conveyerPartsObj){
    geometry_msgs::Pose estimated_conveyor_pose = product.estimated_conveyor_pose;
    conveyor_up_.gantry = {estimated_conveyor_pose.position.x + 0.1, -(estimated_conveyor_pose.position.y - 0.6) , PI/2};
    goToPresetLocation(conveyor_up_);

    activateGripper("left_arm");
    auto left_gripper_status = getGripperState("left_arm");
    
    while(!left_gripper_status.enabled){
        activateGripper("left_arm");
        left_gripper_status = getGripperState("left_arm");
    }

    geometry_msgs::Pose pickup_pose, pre_pickup_pose;
    auto currentPose = left_arm_group_.getCurrentPose().pose;

    pickup_pose.position.x = estimated_conveyor_pose.position.x;
    pickup_pose.position.y = estimated_conveyor_pose.position.y;
    pickup_pose.position.z = estimated_conveyor_pose.position.z + model_height[product.type] + GRIPPER_HEIGHT - EPSILON;
    pickup_pose.orientation.x = currentPose.orientation.x;
    pickup_pose.orientation.y = currentPose.orientation.y;
    pickup_pose.orientation.z = currentPose.orientation.z;
    pickup_pose.orientation.w = currentPose.orientation.w;

    pre_pickup_pose = pickup_pose;
    float pre_z = 0.03;
    pre_pickup_pose.position.z += pre_z;

    left_arm_group_.setPoseTarget(pre_pickup_pose);
    left_arm_group_.move();  // Move to the pre pick up location
    left_arm_group_.setPoseTarget(pickup_pose);

    bool missed = false;
    int starttime=ros::Time::now().toSec();
    int conveyorCounter = 0;
    while (!left_gripper_status.attached && conveyorCounter < 4) {
        ROS_WARN_STREAM_THROTTLE(1, "Waiting for part to be picked : " << conveyorCounter );
        activateGripper("left_arm");
        if(conveyerPartsObj.checkForPick()) {
            // ROS_DEBUG_STREAM("Trying to pick it up");
            ROS_WARN_STREAM("Lets pick !!!! ");
            left_arm_group_.move();
            left_arm_group_.setPoseTarget(pre_pickup_pose);
            left_arm_group_.move();
            // break;
            left_gripper_status = getGripperState("left_arm");
            if(!left_gripper_status.attached) {
                starttime=ros::Time::now().toSec();
                missed = true;
            }
        }
        if(ros::Time::now().toSec() - starttime >= 15){
            starttime=ros::Time::now().toSec();
            missed = true;
        }

        if(missed == true){
            conveyorCounter++;
            // goToPresetLocation(start_);
            ROS_DEBUG_STREAM_THROTTLE(1, "----MISSED----");
            bool status = false;
            left_arm_group_.setPoseTarget(pre_pickup_pose);
            left_arm_group_.move();
            int starttime1=ros::Time::now().toSec();
            do{
                if(ros::Time::now().toSec() - starttime1 > 15){
                    break;
                }
                ROS_DEBUG_STREAM_THROTTLE(0.5, "----Searching on  conveyor----");
                status = conveyerPartsObj.giveClosestPart(product.type, product.estimated_conveyor_pose);
            }while(status != true);
            estimated_conveyor_pose = product.estimated_conveyor_pose;
            conveyor_up_.gantry = {estimated_conveyor_pose.position.x + 0.1, -(estimated_conveyor_pose.position.y - 0.6) , PI/2};
            goToPresetLocation(conveyor_up_);

            pickup_pose.position.x = estimated_conveyor_pose.position.x;
            pickup_pose.position.y = estimated_conveyor_pose.position.y;
            pickup_pose.position.z = estimated_conveyor_pose.position.z + model_height[product.type] + GRIPPER_HEIGHT - EPSILON;
            pickup_pose.orientation.x = currentPose.orientation.x;
            pickup_pose.orientation.y = currentPose.orientation.y;
            pickup_pose.orientation.z = currentPose.orientation.z;
            pickup_pose.orientation.w = currentPose.orientation.w;

            pre_pickup_pose = pickup_pose;
            pre_pickup_pose.position.z += pre_z;

            left_arm_group_.setPoseTarget(pre_pickup_pose);
            left_arm_group_.move();  // Move to the pre pick up location
            left_arm_group_.setPoseTarget(pickup_pose);
            missed = false;
        }
        
        left_gripper_status = getGripperState("left_arm");
    }
    left_gripper_status = getGripperState("left_arm");
    if(!left_gripper_status.attached){
        return false;
    }



    left_arm_group_.setPoseTarget(currentPose);
    left_arm_group_.move();


    tf2::Quaternion q_conveyor( 0, 0, -0.7068252, 0.7073883); // -PI/2
    tf2::Quaternion q_init_part_temp(estimated_conveyor_pose.orientation.x,
                                estimated_conveyor_pose.orientation.y,
                                estimated_conveyor_pose.orientation.z,
                                estimated_conveyor_pose.orientation.w);

    tf2::Quaternion q_res = q_init_part_temp*q_conveyor;
    product.p.pose.orientation.x = q_res.x();
    product.p.pose.orientation.y = q_res.y();
    product.p.pose.orientation.z = q_res.z();
    product.p.pose.orientation.w = q_res.w();
    return true;
}



void GantryControl::goToPresetLocation(PresetLocation location) {
    //--gantry
    joint_group_positions_.at(0) = location.gantry.at(0);
    joint_group_positions_.at(1) = location.gantry.at(1);
    joint_group_positions_.at(2) = location.gantry.at(2);
    //--left arm
    joint_group_positions_.at(3) = location.left_arm.at(0);
    joint_group_positions_.at(4) = location.left_arm.at(1);
    joint_group_positions_.at(5) = location.left_arm.at(2);
    joint_group_positions_.at(6) = location.left_arm.at(3);
    joint_group_positions_.at(7) = location.left_arm.at(4);
    joint_group_positions_.at(8) = location.left_arm.at(5);
    //--right arm
    joint_group_positions_.at(9) = location.right_arm.at(0);
    joint_group_positions_.at(10) = location.right_arm.at(1);
    joint_group_positions_.at(11) = location.right_arm.at(2);
    joint_group_positions_.at(12) = location.right_arm.at(3);
    joint_group_positions_.at(13) = location.right_arm.at(4);
    joint_group_positions_.at(14) = location.right_arm.at(5);

    full_robot_group_.setJointValueTarget(joint_group_positions_);


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (full_robot_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();
}


int GantryControl::getNearestGap(float destX, int aisle_num, bool actPart, ObstaclesInAisle &obstObj,
                                const std::vector< std::pair<float , float> > &shelfGaps){
    int shelf_rwo_left, shelf_row_right;
    switch(aisle_num){
        case 1: shelf_rwo_left = 0;
                shelf_row_right = 1;
                if(actPart == 0)
                    shelf_row_right++;
                break;
        case 2: shelf_rwo_left = 1;
                shelf_row_right = 2;
                if(actPart == 0)
                    shelf_row_right++;
                break;
        case 3: shelf_rwo_left = 2;
                shelf_row_right = 3;
                if(actPart == 0)
                    shelf_row_right++;
                break;
        case 4: shelf_rwo_left = 3;
                shelf_row_right = 4;
    }

    int nearestGap = -1;
    if(std::abs(shelfGaps[shelf_rwo_left].first - destX) < std::abs(shelfGaps[shelf_row_right].first - destX) ){
        nearestGap = shelf_rwo_left;
    }else if(std::abs(shelfGaps[shelf_rwo_left].first - destX) == std::abs(shelfGaps[shelf_row_right].first - destX)){
        if(obstObj.isAisleClear(shelf_rwo_left)){
            nearestGap = shelf_rwo_left;
        }else if(obstObj.isAisleClear(shelf_rwo_left + 1)){
            nearestGap = shelf_rwo_left;
        }else{
            nearestGap = shelf_row_right;
        }
    }else{
        nearestGap = shelf_row_right;
    }
    return nearestGap;
}

bool GantryControl::escape(int &aisle_num, std::vector< std::pair<float , float> > &shelfGaps, const std::vector<int> &gapNum,
                            bool actPart, float &gantryX, float &gantryY, ObstaclesInAisle &obstObj, int &currGap,
                            std::vector<double> &left_arm, bool pickStatus){
    
    int org_aisle_num = aisle_num;
    shelfGaps[0].first = gantryX;
    shelfGaps[4].first = gantryX;
    shelfGaps[0].second = 6;
    shelfGaps[4].second = -6;
    auto left_arm_org = left_arm;
    // get the nearest gap from the end, as parts are at the end helf only
    int nearestGap = getNearestGap(-18, aisle_num, actPart, obstObj, shelfGaps);
    ROS_WARN_STREAM("escape() Shelf row: " << nearestGap);
    PresetLocation temp = start_;
    temp.left_arm = left_arm;
    if(temp.left_arm[2] > 0){
        temp.left_arm[1] = -PI - PI/4;
    }else{
        temp.left_arm[1] = PI/4;
    }
    left_arm = temp.left_arm;
    if(actPart == 1){
        temp.gantry[0] = shelfGaps[nearestGap].first;
         if(nearestGap != 0 && nearestGap != 4){
            if(shelfGaps[nearestGap].second > gantryY) {
                temp.gantry[1] = -(shelfGaps[nearestGap].second + shelfGaps[nearestGap + 1].second)/2;
            }else{
                temp.gantry[1] = -(shelfGaps[nearestGap - 1].second + shelfGaps[nearestGap].second)/2;
            }
            temp.right_arm = { PI, 0, 0, 0, 0, 0};
            goToPresetLocation(temp);
        }

        temp.gantry[0] = shelfGaps[nearestGap].first;
        temp.gantry[1] = -shelfGaps[nearestGap].second;
        goToPresetLocation(temp);

        gantryX = temp.gantry[0];
        gantryY = -temp.gantry[1];
        currGap = gapNum[nearestGap];
        aisle_num = nearestGap;
        if(pickStatus == false){
            aisle_num = org_aisle_num;
            return false;
        }

        if(nearestGap == 0 || nearestGap == 4){
            return true;
        }
    }

    if(actPart == 0){
        int destination_gap = gapNum[nearestGap];
        float destination_x = shelfGaps[nearestGap].first;
        int new_aisle = aisle_num;
        if(nearestGap > new_aisle){
            new_aisle = nearestGap;
        }
        bool move = false;
        do{
            move = obstObj.moveBot(destination_x, destination_gap, new_aisle, gantryX, currGap);

        }while(!move);

        temp.gantry[1] = -(shelfGaps[new_aisle - 1].second + shelfGaps[new_aisle].second)/2;
        temp.gantry[0] = gantryX;
        temp.right_arm = { PI, 0, 0, 0, 0, 0};
        goToPresetLocation(temp);
        temp.gantry[0] = shelfGaps[nearestGap].first;
        goToPresetLocation(temp);

        temp.gantry[1] = -shelfGaps[nearestGap].second;
        goToPresetLocation(temp);

        gantryX = temp.gantry[0];
        gantryY = -temp.gantry[1];
        if(nearestGap == 0 || nearestGap == 4){
            aisle_num = nearestGap ? nearestGap : nearestGap+1;
            currGap = gapNum[nearestGap];
            return true;
        }
            
    }

    aisle_num = nearestGap;
    currGap = gapNum[nearestGap];

    if(obstObj.isAisleClear(aisle_num)){
        temp.gantry[0] = gantryX;
        temp.gantry[1] = -(shelfGaps[aisle_num-1].second + shelfGaps[aisle_num].second)/2;
        temp.right_arm = { PI, 0, 0, 0, 0, 0};
        goToPresetLocation(temp);

        gantryX = temp.gantry[0];
        gantryY = -temp.gantry[1];        
        return true;
    }

    if(obstObj.isAisleClear(aisle_num + 1)){
        temp.gantry[0] = gantryX;
        temp.gantry[1] = -(shelfGaps[aisle_num].second + shelfGaps[aisle_num + 1].second)/2;
        temp.right_arm = { PI, 0, 0, 0, 0, 0};
        goToPresetLocation(temp);
        gantryX = temp.gantry[0];
        gantryY = -temp.gantry[1];
        aisle_num += 1;     
        return true;
    }

    left_arm = left_arm_org;
    escape(aisle_num, shelfGaps, gapNum, 0, gantryX, gantryY, obstObj, currGap, left_arm, pickStatus);
    return true;
}

bool GantryControl::move2closestGap(struct Part &part, std::vector< std::pair<float , float> > &shelfGaps,
                                    const std::vector<int> &gapNum, bool actPart, float &gantryX, float &gantryY,
                                    ObstaclesInAisle &obstObj, int &newGap, std::vector<double> &left_arm){
    int aisle_num = part.aisle_num;
    shelfGaps[0].first = part.pose.position.x;
    shelfGaps[4].first = part.pose.position.x;
    shelfGaps[0].second = 6;
    shelfGaps[4].second = -6;

    int poseXtemp = part.pose.position.x;
    if(actPart == 1){
        poseXtemp = -18;
    }

    int nearestGap = getNearestGap(poseXtemp, aisle_num, actPart, obstObj, shelfGaps);

    ROS_WARN_STREAM("move2closestGap() Shelf row = " << nearestGap);
    float offset_y = 1.1;
    float offset_x = 0;

    if(actPart == 0){
        offset_y += 0.3; // when the y is of the intermediate gap, and not of part
                         // need to consider the offset of shelf center upto part center displacement
        offset_x = 0;
    }
    
    if(part.pose.position.y < 0){
        offset_y *= -1;
    }
    PresetLocation temp = start_;
    if(nearestGap == 0 || nearestGap == 4){
        temp.gantry[0] = 0.4;
        temp.gantry[1] = -shelfGaps[nearestGap].second;
        goToPresetLocation(temp);
        
        temp.gantry[0] = shelfGaps[nearestGap].first - offset_x;
        temp.gantry[1] = -shelfGaps[nearestGap].second;
        if(nearestGap == 0){
            temp.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};
        }
        else{
            temp.left_arm = {-PI/2 , -PI/2, -PI/2 - PI/4 , -PI/2 - PI/4, 0, 0};
        }
        goToPresetLocation(temp);
        gantryX = temp.gantry[0];
        gantryY = -temp.gantry[1];
        if(nearestGap == 0)
            part.aisle_num = nearestGap + 1;
        else
            part.aisle_num = nearestGap;
        left_arm = temp.left_arm;
        return true;
    }

    float gap_offset_y = 1.5;
    if(obstObj.isAisleClear(nearestGap)){
        if(nearestGap >= 3){
            gap_offset_y *= -1;
        }
        temp.gantry[0] = 0.4;
        temp.gantry[1] = -(shelfGaps[nearestGap].second + gap_offset_y);
        goToPresetLocation(temp);
        
        temp.gantry[0] = shelfGaps[nearestGap].first;
        temp.gantry[1] = -(shelfGaps[nearestGap].second + gap_offset_y);
        temp.left_arm = { 0, 0, 0, 0, 0, 0};
        temp.right_arm = { PI, 0, 0, 0, 0, 0};
        goToPresetLocation(temp);

        temp.gantry[0] = shelfGaps[nearestGap].first;
        temp.gantry[1] = -(shelfGaps[nearestGap].second);
        goToPresetLocation(temp);
        gantryX = temp.gantry[0];
        gantryY = -temp.gantry[1];

        part.aisle_num = nearestGap + 1;
        newGap = gapNum[nearestGap];

        left_arm = temp.left_arm;
        return true;
    }

    if(obstObj.isAisleClear(nearestGap + 1)){
        if(nearestGap + 1 >= 3){
            gap_offset_y *= -1;
        }
        temp.gantry[0] = 0.4;
        temp.gantry[1] = -(shelfGaps[nearestGap].second + gap_offset_y);
        goToPresetLocation(temp);
        
        temp.gantry[0] = shelfGaps[nearestGap].first;
        temp.gantry[1] = -(shelfGaps[nearestGap].second + gap_offset_y);
        temp.left_arm = { 0, 0, 0, 0, 0, 0};
        temp.right_arm = { PI, 0, 0, 0, 0, 0};
        goToPresetLocation(temp);

        temp.gantry[0] = shelfGaps[nearestGap].first;
        temp.gantry[1] = -(shelfGaps[nearestGap].second);
        goToPresetLocation(temp);

        gantryX = temp.gantry[0];
        gantryY = -temp.gantry[1];
        part.aisle_num = nearestGap;
        newGap = gapNum[nearestGap];
        left_arm = temp.left_arm;
        return true;
    }

    Part fakePart;
    fakePart.pose.position.x = shelfGaps[nearestGap].first;
    fakePart.pose.position.y = shelfGaps[nearestGap].second;
    fakePart.aisle_num = nearestGap;
    int newGap_ = -1;

    move2closestGap(fakePart, shelfGaps, gapNum, 0, gantryX, gantryY, obstObj, newGap_, left_arm);

    bool move = false;
    // ROS_WARN_STREAM("Sensor checked in aisle: " << fakePart.aisle_num);
    do{
        move = obstObj.moveBot(fakePart.pose.position.x, gapNum[nearestGap], fakePart.aisle_num, gantryX, newGap_);

    }while(!move);


    temp.gantry[0] = gantryX;
    temp.gantry[1] = -(fakePart.pose.position.y + gantryY)/2;
    temp.left_arm = { 0, 0, 0, 0, 0, 0};
    temp.right_arm = { PI, 0, 0, 0, 0, 0};
    goToPresetLocation(temp);
    
    temp.gantry[0] = fakePart.pose.position.x;
    goToPresetLocation(temp);

    temp.gantry[1] = -fakePart.pose.position.y;
    goToPresetLocation(temp);

    gantryX = temp.gantry[0];
    gantryY = -temp.gantry[1];
    newGap = gapNum[nearestGap];
    left_arm = temp.left_arm;
    return true;
}

/// Turn on vacuum gripper
void GantryControl::activateGripper(std::string arm_name) {
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = true;

    if (arm_name == "left_arm") {
        left_gripper_control_client.call(srv);
    } else {
        right_gripper_control_client.call(srv);
    }
    // ROS_INFO_STREAM("[GantryControl][activateGripper] DEBUG: srv.response =" << srv.response);
}

/// Turn off vacuum gripper
void GantryControl::deactivateGripper(std::string arm_name) {
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = false;

    if (arm_name == "left_arm") {
        left_gripper_control_client.call(srv);
    } else {
        right_gripper_control_client.call(srv);
    }
    ROS_INFO_STREAM("[GantryControl][deactivateGripper] DEBUG: srv.response =" << srv.response);
}

/// Retrieve gripper state
nist_gear::VacuumGripperState GantryControl::getGripperState(std::string arm_name) {
    if (arm_name == "left_arm") {
        return current_left_gripper_state_;
    } else {
        return current_right_gripper_state_;
    }
}

/// Called when a new VacuumGripperState message is received
void GantryControl::left_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & gripper_state_msg) {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gripper States (throttled to 0.1 Hz):\n" << *gripper_state_msg);
    current_left_gripper_state_ = *gripper_state_msg;
}

void GantryControl::right_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & gripper_state_msg) {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gripper States (throttled to 0.1 Hz):\n" << *gripper_state_msg);
    current_right_gripper_state_ = *gripper_state_msg;
}

/// Called when a new JointState message is received
void GantryControl::joint_states_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg) {
    if (joint_state_msg->position.size() == 0) {
        ROS_ERROR("[gantry_control][joint_states_callback] msg->position.size() == 0!");
    }
    current_joint_states_ = *joint_state_msg;
}


void GantryControl::gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg) {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gantry controller states (throttled to 0.1 Hz):\n" << *msg);
    current_gantry_controller_state_ = *msg;
}

void GantryControl::left_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg) {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Left arm controller states (throttled to 0.1 Hz):\n" << *msg);
    current_left_arm_controller_state_ = *msg;
}

void GantryControl::right_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg) {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Right arm controller states (throttled to 0.1 Hz):\n" << *msg);
    current_right_arm_controller_state_ = *msg;
}


bool GantryControl::send_command(trajectory_msgs::JointTrajectory command_msg) {
    // ROS_INFO_STREAM("[gantry_control][send_command] called.");

    if(command_msg.points.size() == 0) {
        ROS_WARN("[gantry_control][send_command] Trajectory is empty or NAN, returning.");
        return false;
    }
    else if ((command_msg.joint_names[0] == "small_long_joint") // command is for gantry
             && (command_msg.points[0].positions[0] == command_msg.points[0].positions[0])) {

        gantry_joint_trajectory_publisher_.publish(command_msg);
        // ROS_INFO_STREAM("[gantry_control][send_command] gantry command published!");
        return true;
    }
    else if ((command_msg.joint_names[0].substr(0,4) == "left") // command is for left_arm
             && (command_msg.points[0].positions[0] == command_msg.points[0].positions[0])) {

        left_arm_joint_trajectory_publisher_.publish(command_msg);
        // ROS_INFO_STREAM("[gantry_control][send_command] left_arm command published!");
        return true;
    }
    else if ((command_msg.joint_names[0].substr(0,5) == "right") // command is for right arm
             && (command_msg.points[0].positions[0] == command_msg.points[0].positions[0])) {

        right_arm_joint_trajectory_publisher_.publish(command_msg);
        // ROS_INFO_STREAM("[gantry_control][send_command] right_arm command published!");
        return true;
    }
    else {
        return false;
    }
}

