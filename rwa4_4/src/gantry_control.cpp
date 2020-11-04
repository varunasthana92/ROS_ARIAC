#include "gantry_control.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nist_gear/LogicalCameraImage.h>
#include "utils.h"
#include "conveyer.h"


// Quality control sensor 1 callback
void GantryControl::qualityCallback1(const nist_gear::LogicalCameraImage& msg) {
    if (msg.models.size() != 0) {
        // ROS_INFO_STREAM("Detected faulty part on agv2 : " << (msg.models[0]).type);
        is_part_faulty_agv2 = true;
        geometry_msgs::Pose model_pose = (msg.models[msg.models.size()-1]).pose;
        geometry_msgs::TransformStamped transformStamped;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Duration timeout(1.0);
        bool transform_exists = tfBuffer.canTransform("world", "quality_control_sensor_1_frame", ros::Time(0), timeout);
        if (transform_exists)
            transformStamped = tfBuffer.lookupTransform("world", "quality_control_sensor_1_frame", ros::Time(0));
        else
            ROS_INFO_STREAM("Cannot transform from quality_control_sensor_1_frame to world");
        geometry_msgs::PoseStamped new_pose;
        new_pose.header.seq = 1;
        new_pose.header.stamp = ros::Time(0);
        new_pose.header.frame_id = "quality_control_sensor_1_frame";
        new_pose.pose = model_pose;
        tf2::doTransform(new_pose, new_pose, transformStamped);
        faulty_part_pose_agv2 = new_pose.pose;       
    }
}

void GantryControl::qualityCallback2(const nist_gear::LogicalCameraImage& msg) {
    if (msg.models.size() != 0) {
        // ROS_INFO_STREAM("Detected faulty part n agv1 : " << (msg.models[0]).type);
        is_part_faulty_agv1 = true;
        geometry_msgs::Pose model_pose = (msg.models[msg.models.size()-1]).pose;
        geometry_msgs::TransformStamped transformStamped;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Duration timeout(1.0);
        bool transform_exists = tfBuffer.canTransform("world", "quality_control_sensor_2_frame", ros::Time(0), timeout);
        if (transform_exists)
            transformStamped = tfBuffer.lookupTransform("world", "quality_control_sensor_2_frame", ros::Time(0));
        else
            ROS_INFO_STREAM("Cannot transform from quality_control_sensor_2_frame to world");
        geometry_msgs::PoseStamped new_pose;
        new_pose.header.seq = 1;
        new_pose.header.stamp = ros::Time(0);
        new_pose.header.frame_id = "quality_control_sensor_2_frame";
        new_pose.pose = model_pose;
        tf2::doTransform(new_pose, new_pose, transformStamped);
        faulty_part_pose_agv1 = new_pose.pose;       
    }
}

void GantryControl::logicalCallback16(const nist_gear::LogicalCameraImage& msg) {
    for(auto curr_model : msg.models){
        // ROS_INFO_STREAM("Detected part from logical camera 16 on agv1: " << (msg.models[0]).type);
        geometry_msgs::Pose model_pose = curr_model.pose;
        std::string model_name = curr_model.type;

        geometry_msgs::TransformStamped transformStamped;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Duration timeout(1.0);
        bool transform_exists = tfBuffer.canTransform("world", "logical_camera_16_frame", ros::Time(0), timeout);
        if (transform_exists)
            transformStamped = tfBuffer.lookupTransform("world", "logical_camera_16_frame", ros::Time(0));
        else
            ROS_INFO_STREAM("Cannot transform from logical_camera_16_frame to world");
        geometry_msgs::Pose world_pose;
        tf2::doTransform(model_pose, world_pose, transformStamped);

        if(not check_exist_on_agv(model_name, world_pose, agv1_allParts)){
            part_placed_pose_agv1 = world_pose;
            return;
        }
        /*
        ROS_INFO_STREAM("Incorrect part pose: " << part_placed_pose_agv1.position.x << std::endl
                                                << part_placed_pose_agv1.position.y << std::endl
                                                << part_placed_pose_agv1.position.z << std::endl
                                                << part_placed_pose_agv1.orientation.x << std::endl
                                                << part_placed_pose_agv1.orientation.y << std::endl
                                                << part_placed_pose_agv1.orientation.z << std::endl
                                                << part_placed_pose_agv1.orientation.w)
        */
    }
}

void GantryControl::logicalCallback17(const nist_gear::LogicalCameraImage& msg) {
    for(auto curr_model : msg.models){
        // ROS_INFO_STREAM("Detected part from logical camera: " << (msg.models[0]).type);

        geometry_msgs::Pose model_pose = curr_model.pose;
        std::string model_name = curr_model.type;

        geometry_msgs::TransformStamped transformStamped;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Duration timeout(1.0);
        bool transform_exists = tfBuffer.canTransform("world", "logical_camera_17_frame", ros::Time(0), timeout);
        if (transform_exists)
            transformStamped = tfBuffer.lookupTransform("world", "logical_camera_17_frame", ros::Time(0));
        else
            ROS_INFO_STREAM("Cannot transform from logical_camera_17_frame to world");
        geometry_msgs::Pose world_pose;
        tf2::doTransform(model_pose, world_pose, transformStamped);

        if(not check_exist_on_agv(model_name, world_pose, agv1_allParts)){
            part_placed_pose_agv2 = world_pose;
            return;
        }
    }
}

float getDis(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2){

    if(std::abs(p1.position.z - p2.position.z) > 0.004){
        return -1;
    }
    float dis = std::pow((p1.position.x - p2.position.x),2) + std::pow((p1.position.y - p2.position.y),2);
    return std::sqrt(dis);
}

bool GantryControl::check_exist_on_agv(const std::string &name, const geometry_msgs::Pose &part_pose, agvInfo &agv){
    if(agv.prod_on_tray.find(name) != agv.prod_on_tray.end()){
        for(Product prod: agv.prod_on_tray[name]){
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
    ROS_INFO_STREAM("Pose 1:" << pose1.position.x << std::endl
                              << pose1.position.y << std::endl
                              << pose1.position.z << std::endl
                              << pose1.orientation.x << std::endl
                              << pose1.orientation.y << std::endl
                              << pose1.orientation.w << std::endl
                              << pose1.orientation.z);
    ROS_INFO_STREAM("Pose 2:" << pose2.position.x << std::endl
                              << pose2.position.y << std::endl
                              << pose2.position.z << std::endl
                              << pose2.orientation.x << std::endl
                              << pose2.orientation.y << std::endl
                              << pose2.orientation.w << std::endl
                              << pose2.orientation.z);


    std::vector<double> pose1_angles = quaternionToEuler(pose1);
    std::vector<double> pose2_angles = quaternionToEuler(pose2);

    ROS_INFO_STREAM("Yaw values:" << pose1_angles.at(2) << std::endl
                                  << pose2_angles.at(2));

    ROS_INFO_STREAM("Difference:" << std::abs(pose1.position.x - pose2.position.x) << std::endl
                              << std::abs(pose1.position.y - pose2.position.y) << std::endl
                              << std::abs(pose1_angles.at(2) - pose2_angles.at(2)));
    if (std::abs(pose1.position.x - pose2.position.x) < 0.03 &&
        std::abs(pose1.position.y - pose2.position.y) < 0.03
        //std::abs(pose1_angles.at(2) - pose2_angles.at(2)) < 0.1
        )
        return true;
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

    agv1_.gantry = {-0.6, -6.9, 0.};
    agv1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv1_drop.gantry = {0, -5, 0.};
    agv1_drop.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv1_drop.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv1_right_.gantry = {0.6, -6.9, 0};
    agv1_right_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv1_right_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv2_.gantry = {0.6, 6.9, PI};
    agv2_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv2_drop.gantry = {0, 5, PI};
    agv2_drop.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv2_drop.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv2_right_.gantry = {-0.6, 6.9, PI};
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

    conveyor_up_.gantry = {0, 1.2, 1.29};
    conveyor_up_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
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
    ros::Duration timeout(5.0);


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
    goToPresetLocation(flipped_pulley_preset);
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
    part.pose.position.z = part.pose.position.z + model_height.at(part.type) + GRIPPER_HEIGHT - EPSILON;

    ROS_WARN_STREAM("robot quat: "<< currentPose);
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
        part.pose.position.z += 0.2;
        left_arm_group_.setPoseTarget(part.pose);
        left_arm_group_.move();
        part.pose.position.z -= 0.2;
        left_arm_group_.setPoseTarget(part.pose);
        left_arm_group_.move();
        auto state = getGripperState("left_arm");
        if (state.attached) {
            ROS_INFO_STREAM("[Gripper] = object attached");
            //--Move arm to previous position
            part.pose.position.z += 0.2;
            left_arm_group_.setPoseTarget(part.pose);
            left_arm_group_.move();

            return true;
            // part.pose.position.z -= 0.2;
            left_arm_group_.setPoseTarget(currentPose);
            left_arm_group_.move();
            // goToPresetLocation(start_);
        }
        else {
            ROS_INFO_STREAM("[Gripper] = object not attached");
            if (*is_part_faulty)
                part.pose = *faulty_part_pose;
            int max_attempts{5};
            int current_attempt{0};
            while(!state.attached) {
                part.pose.position.z += 0.2;
                left_arm_group_.setPoseTarget(part.pose);
                left_arm_group_.move();
                part.pose.position.z -= 0.2;
                // left_arm_group_.setPoseTarget(par);
                // left_arm_group_.move();
                ros::Duration(0.5).sleep();
                left_arm_group_.setPoseTarget(part.pose);
                left_arm_group_.move();
                activateGripper("left_arm");
            }
        }
        return true;
    }
    else {
        ROS_INFO_STREAM("[Gripper] = not enabled");
    }
    return false;
    
    /**
     * We want the Cartesian path to be interpolated at a resolution of 1 cm which is why
     * we will specify 0.01 as the max step in Cartesian translation.
     * We will specify the jump threshold as 0.0, effectively disabling it.
     */
    //--define a set of waypoints
//    geometry_msgs::Pose near_pick_pose;
//    geometry_msgs::Pose pick_pose;
//    near_pick_pose = part.pose;
//    pick_pose = part.pose;
//
//    near_pick_pose.position.z += 0.1;
//    pick_pose.position.z += 0.015;
//
//    //--waypoints
//    ROS_INFO_STREAM("[near_pick_pose]= " << near_pick_pose.position.x << "," << near_pick_pose.position.y << "," << near_pick_pose.position.z << "," << near_pick_pose.orientation.x << "," << near_pick_pose.orientation.y << "," << near_pick_pose.orientation.z << "," << near_pick_pose.orientation.w);
//    ROS_INFO_STREAM("[pick_pose]= " << pick_pose.position.x << "," << pick_pose.position.y << "," << pick_pose.position.z << "," << pick_pose.orientation.x << "," << pick_pose.orientation.y << "," << pick_pose.orientation.z << "," << pick_pose.orientation.w);
//    std::vector<geometry_msgs::Pose> waypoints;
//    waypoints.push_back(near_pick_pose);
//    waypoints.push_back(pick_pose);

//    moveit_msgs::RobotTrajectory trajectory;
//    const double jump_threshold = 0.0;
//    const double eef_step = 0.001;
//    double fraction = left_arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
//
//    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//    bool success = (left_arm_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//    if (success)
//        left_arm_group_.move();
//    ros::waitForShutdown();
}

bool GantryControl::placePart(Product &product,
                              std::string agv,
                              std::string arm){
    
    Part part = product.p;
    auto target_pose_in_tray = getTargetWorldPose(product.pose, agv, arm);
    
    product.rpy_final = quaternionToEuler(target_pose_in_tray);

    // double yaw_ = product.rpy_final[2]- part.rpy_init[2];
    // double pitch_ = 0;
    // double roll_ = 0;
    // tf2::Quaternion q_final_part(yaw_, pitch_, roll_);


    tf2::Quaternion q_pitch( 0, 0.7073883, 0, 0.7073883);
    tf2::Quaternion q_pi( 0, 0, 0.9999997, 0.0007963);

    tf2::Quaternion q_init_part(part.pose.orientation.x,
                                part.pose.orientation.y,
                                part.pose.orientation.z,
                                part.pose.orientation.w);

    tf2::Quaternion q_final_part(target_pose_in_tray.orientation.x,
                                target_pose_in_tray.orientation.y,
                                target_pose_in_tray.orientation.z,
                                target_pose_in_tray.orientation.w);

    tf2::Quaternion q_rslt = q_init_part.inverse()*q_final_part*q_pi*q_pitch;

    target_pose_in_tray.orientation.x = q_rslt.x();
    target_pose_in_tray.orientation.y = q_rslt.y();
    target_pose_in_tray.orientation.z = q_rslt.z();
    target_pose_in_tray.orientation.w = q_rslt.w();
    
    ROS_INFO_STREAM("Settled tray pose:" << target_pose_in_tray.position.x << " " 
                                            << target_pose_in_tray.position.y << " "
                                            << target_pose_in_tray.position.z);
//    ros::Duration(3.0).sleep();
    auto left_state = getGripperState("left_arm");
    auto right_state = getGripperState("right_arm");
    target_pose_in_tray.position.z += (ABOVE_TARGET + 1.5*model_height[part.type]);
    geometry_msgs::Pose currentPose;
    PresetLocation agv_in_use;
    PresetLocation agv_in_use_drop;
    PresetLocation agv_in_use_right;
    struct agvInfo* agv_data;
    if(agv == "agv1"){
        agv_in_use = agv1_;
        agv_in_use_drop = agv1_drop;
        agv_in_use_right = agv1_right_;
        agv_data = &agv1_allParts;
    }else{
        agv_in_use = agv2_;
        agv_in_use_drop = agv2_drop;
        agv_in_use_right = agv2_right_;
        agv_data = &agv2_allParts;
    }
    if (left_state.attached) {
        goToPresetLocation(agv_in_use);
        
        currentPose = left_arm_group_.getCurrentPose().pose;

        left_arm_group_.setPoseTarget(target_pose_in_tray);
        left_arm_group_.move();

        deactivateGripper("left_arm");
        left_arm_group_.setPoseTarget(currentPose);
        left_arm_group_.move();
    } else if (right_state.attached){

//        auto robot_rpy = quaternionToEuler(currentPose);
//        float roll_ = robot_rpy[0];
//        float pitch_ = robot_rpy[1];
//        float yaw_ = robot_rpy[2];
//        auto temp_present = agv_in_use_right;
//
//        yaw_ = product.rpy_final[2]- part.rpy_init[2];
//
//        temp_present.right_arm[5] = yaw_;

        goToPresetLocation(agv_in_use_right);
        currentPose = left_arm_group_.getCurrentPose().pose;
        // auto robot_rpy = quaternionToEuler(currentPose);
        // float roll_ = robot_rpy[0];
        // float pitch_ = robot_rpy[1];
        // float yaw_ = robot_rpy[2];

        // yaw_ = product.rpy_final[2]- part.rpy_init[2];

        // tf2::Quaternion q_robot_new(yaw_, pitch_, roll_);

        // target_pose_in_tray.orientation.x = q_robot_new.x();
        // target_pose_in_tray.orientation.y = q_robot_new.y();
        // target_pose_in_tray.orientation.z = q_robot_new.z();
        // target_pose_in_tray.orientation.w = q_robot_new.w();

        target_pose_in_tray.orientation.x = currentPose.orientation.x;
        target_pose_in_tray.orientation.y = currentPose.orientation.y;
        target_pose_in_tray.orientation.z = currentPose.orientation.z;
        target_pose_in_tray.orientation.w = currentPose.orientation.w;

        right_arm_group_.setPoseTarget(target_pose_in_tray);
        right_arm_group_.move();

        deactivateGripper("right_arm");
        right_arm_group_.setPoseTarget(currentPose);
        right_arm_group_.move();
    }
    
    //bool is_part_placed_correct = poseMatches(target_pose_in_tray, part_placed_pose_incorrect)
    bool *is_part_faulty;
    geometry_msgs::Pose* part_placed_pose;
    geometry_msgs::Pose* faulty_part_pose;

    if(part.agv_id == "agv2"){
    	ROS_INFO_STREAM("------Setting to agv2");
    	is_part_faulty = &is_part_faulty_agv2;
    	faulty_part_pose = &faulty_part_pose_agv2;
        part_placed_pose = &part_placed_pose_agv2;
    }else{
    	ROS_INFO_STREAM("------Setting to agv1");
    	is_part_faulty = &is_part_faulty_agv1;
    	faulty_part_pose = &faulty_part_pose_agv1;
        part_placed_pose = &part_placed_pose_agv1;
    }

    ros::Duration(2).sleep();
    if (*is_part_faulty) {
        ROS_INFO_STREAM("-----------------Part faulty inside: " << *is_part_faulty);
        part.pose = *faulty_part_pose;
        pickPart(part);
        *is_part_faulty = false;
        goToPresetLocation(agv_in_use);
        goToPresetLocation(agv_in_use_drop);
        deactivateGripper("left_arm");
        return false;
    } 
    bool is_part_placed_correct = poseMatches(target_pose_in_tray, *part_placed_pose);
    if(is_part_placed_correct){
        product.agv_world_pose = *part_placed_pose;
        product.part_placed = true;
        agv_data->prod_on_tray[product.type].push_back(product);
        agv_data->count++;
    }else{
        ROS_INFO_STREAM("Part placed incorrectly: " << is_part_placed_correct);
        geometry_msgs::Pose original_part_pose = part.pose;
        part.pose = *part_placed_pose;
        ROS_INFO_STREAM("Incorrect part pose: " << part.pose.position.x << std::endl
                                                << part.pose.position.y << std::endl
                                                << part.pose.position.z << std::endl
                                                << part.pose.orientation.x << std::endl
                                                << part.pose.orientation.y << std::endl
                                                << part.pose.orientation.z << std::endl
                                                << part.pose.orientation.w);
        pickPart(part);
        // part.pose = original_part_pose;
        // ROS_INFO_STREAM("Original part pose: " << part.pose.position.x << std::endl
        //                                         << part.pose.orientation.x << std::endl
        //                                         << part.pose.orientation.y << std::endl
        //                                         << part.pose.orientation.z << std::endl
        //                                         << part.pose.orientation.w);
        bool placed = placePart(product, agv, arm);
    }
    return true;
}

bool GantryControl::move2start ( float x, float y ) {

    // geometry_msgs::Pose robot_pose = getRobotPose();
    // float x = curr_pose.position.x;
    // float y = curr_pose.position.y;
    ROS_INFO_STREAM("Position of trg in move2start y:" << y);
    ROS_INFO_STREAM("Position of trg in move2start robot x:" << x);
    ROS_INFO_STREAM("Position of trg in move2start robot x:" << y);
    float offset_y = 0.2;
    float offset_x = 0.2;

    // offset_y *= -1;

    PresetLocation move;
    move.gantry = {x,y,0};
    move.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    move.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    if(x == 0 && y == 0 ){
        return true;
    }

    if(x < 0){
        if( y > 6.6){
            move.gantry[0] = 0;
            move.gantry[1] -= offset_y;
            ROS_INFO_STREAM("Position of trg  in move2start y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  str_1");

            goToPresetLocation(move);
            goToPresetLocation(start_);
            return true;
        }
        else  if( y <= 6.6 && y > 3.05){
            move.gantry[0] = 0;
            move.gantry[1] += offset_y;
            ROS_INFO_STREAM("Position of trg in move2start y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  str_2");

            goToPresetLocation(move);
            goToPresetLocation(start_);
            return true;
        }
        else if( y <= 3.05 && y > 1.5025){
            move.gantry[0] = x;
            move.gantry[1] -= offset_y;
            move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};

            ROS_INFO_STREAM("Position of trg in move2start y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  str_3");

            goToPresetLocation(move);

            move.gantry[0] = 0;
            move.gantry[1] -= offset_y;
            move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};
            goToPresetLocation(move);
//            goToPresetLocation(start_);
            return true;
        }
        else if( y <= 1.5025 && y > 0){
            move.gantry[0] = 0;
            move.gantry[1] += offset_y;

            ROS_INFO_STREAM("Position of trg in move2start y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  str_4");

            goToPresetLocation(move);
            goToPresetLocation(start_);
            return true;
        }
        else if( y <= 0 && y > -1.5025){
            move.gantry[0] = x;
            move.gantry[1] -= offset_y;
            move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};

            ROS_INFO_STREAM("Position of trg in move2start y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  str_5");

            goToPresetLocation(move);

            move.gantry[0] = 0;
            move.gantry[1] -= offset_y;
            move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};
            goToPresetLocation(move);
//            goToPresetLocation(start_);
            return true;
        }
        else if( y <= -1.5025 && y > -3.05){
            move.gantry[0] = 0;
            move.gantry[1] += offset_y;

            ROS_INFO_STREAM("Position of trg in move2start y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  str_6");

            goToPresetLocation(move);
            goToPresetLocation(start_);
            return true;
        }
        else if( y <= -3.05 && y > -6.6){
            move.gantry[0] = x;
            move.gantry[1] -= offset_y;
            move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};
            ROS_INFO_STREAM("Position of trg in move2start x:" << move.gantry[0]);
            ROS_INFO_STREAM("Position of trg in move2start y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  str_7");

            goToPresetLocation(move);

            move.gantry[0] = 0;
            move.gantry[1] -= offset_y;
            move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};
            goToPresetLocation(move);
//            goToPresetLocation(start_);
            return true;
        }
        else if( y <= -6.6){
            move.gantry[0] = 0;
            move.gantry[1] += offset_y;

            ROS_INFO_STREAM("Position of trg in move2start y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  str_8");

            goToPresetLocation(move);
            goToPresetLocation(start_);
            return true;
        }
        return false;

    }else{ // towards bin side
        if( y > 6.6){
            move.gantry[0] = 0;
            move.gantry[1] -= offset_y;

            ROS_INFO_STREAM("Position of trg in move2start y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  str_9");

            goToPresetLocation(move);
            goToPresetLocation(start_);
            return true;
        }
        else  if( y <= 6.6 && y > 3.6){
            move.gantry[0] = 0;
            move.gantry[1] += offset_y;

            ROS_INFO_STREAM("Position of trg in move2start y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  str_10");

            goToPresetLocation(move);
            goToPresetLocation(start_);
            return true;
        }
        else  if( y <= 3.6 && y > 0){
            move.gantry[0] = 0;
            move.gantry[1] -= offset_y;

            ROS_INFO_STREAM("Position of trg in move2start y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  str_11");

            goToPresetLocation(move);
//            goToPresetLocation(start_);
            return true;
        }
        else  if( y <= 0 && y > -3.6){
            move.gantry[0] = 0;
            move.gantry[1] += offset_y;

            ROS_INFO_STREAM("Position of trg in move2start y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  str_12");

            goToPresetLocation(move);
            goToPresetLocation(start_);
            return true;
        }
        else  if( y <= -3.6 && y > -6.6){
            move.gantry[0] = 0;
            move.gantry[1] -= offset_y;

            ROS_INFO_STREAM("Position of trg in move2start y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  str_13");

            goToPresetLocation(move);
            goToPresetLocation(start_);
            return true;
        }
        else  if( y <= -6.6 ){
            move.gantry[0] = 0;
            move.gantry[1] += offset_y;

            ROS_INFO_STREAM("Position of trg in move2start y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  str_14");

            goToPresetLocation(move);
            goToPresetLocation(start_);
            return true;
        }
        return false;
    }
}

float GantryControl::move2trg  ( float x, float y ) {

    // geometry_msgs::Pose robot_pose = getRobotPose();
    // float x = trg_pose.position.x;
    // float y = trg_pose.position.y;

    float offset_final_y = 1.1;
    float offset_y = offset_final_y + 0.2;
    float offset_final_x = 0.4;

    // offset_y *= -1;
    // offset_final_y *= -1;

    ROS_INFO_STREAM("Position of trg in move2trg y:" << y);
    ROS_INFO_STREAM("Position of trg in move2trg offset y:" << offset_y);
    ROS_INFO_STREAM("Position of trg X: " << x);
    ROS_INFO_STREAM("Position of trg Y:" << y);

    PresetLocation move, move_trg;
    move.gantry = {x,y,0};
    move.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    move.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    move_trg = move;

    if(x < 0){
        if( y > 6.3){
            move.gantry[0] = 0;
            move.gantry[1] -= offset_y;
            
            ROS_INFO_STREAM("Position of trg in move2trg y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  trg_1");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] -= offset_final_y;
            goToPresetLocation(move_trg);
            return -move_trg.gantry[1];
        }
        else  if( y <= 6.3 && y > 3.05){
            move.gantry[0] = 0;
            move.gantry[1] += offset_y;

            ROS_INFO_STREAM("Position of trg in move2trg y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  trg_2");

            goToPresetLocation(move);

            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] += offset_final_y;
            goToPresetLocation(move_trg);
            return -move_trg.gantry[1];
        }
        else if( y <= 3.05 && y > 1.5025){
            move.gantry[0] = 0;
            move.gantry[1] -= offset_y;
            move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};
            ROS_INFO_STREAM("Position of trg in move2trg y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  trg_3");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] -= offset_final_y - 0.3;
            move_trg.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};
            goToPresetLocation(move_trg);
            return -move_trg.gantry[1];
        }
        else if( y <= 1.5025 && y > 0){
            move.gantry[0] = 0;
            move.gantry[1] += offset_y;

            ROS_INFO_STREAM("Position of trg in move2trg y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  trg_4");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] += offset_final_y;
            goToPresetLocation(move_trg);
            return -move_trg.gantry[1];
        }
        else if( y <= 0 && y > -1.5025){
            move.gantry[0] = 0;
            move.gantry[1] -= offset_y;
            move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};
            ROS_INFO_STREAM("Position of trg in move2trg y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  trg_5");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] -= offset_final_y - 0.3;
            move_trg.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};
            goToPresetLocation(move_trg);
            return -move_trg.gantry[1];
        }
        else if( y <= -1.5025 && y > -3.05){
            move.gantry[0] = 0;
            move.gantry[1] += offset_y;

            ROS_INFO_STREAM("Position of trg in move2trg y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  trg_6");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] += offset_final_y;
            goToPresetLocation(move_trg);
            return -move_trg.gantry[1];
        }
        else if( y <= -3.05 && y > -6.6){
            move.gantry[0] = 0;
            move.gantry[1] -= offset_y;
            move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};
            ROS_INFO_STREAM("Position of trg in move2trg y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  trg_7");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] -= offset_final_y;
            move_trg.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, -PI/4, 0, 0};
            goToPresetLocation(move_trg);
            return -move_trg.gantry[1];
        }
        else if( y <= -6.6){
            move.gantry[0] = 0;
            move.gantry[1] += offset_y;

            ROS_INFO_STREAM("Position of trg in move2trg y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  trg_8");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] += offset_final_y;
            goToPresetLocation(move_trg);
            return -move_trg.gantry[1];
        }
        return 0;

    }else{ // towards bin s
        if( y > 6.3){
            move.gantry[0] = 0;
            move.gantry[1] -= offset_y;

            ROS_INFO_STREAM("Position of trg in move2trg y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  trg_9");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] -= offset_final_y;
            goToPresetLocation(move_trg);
            return -move_trg.gantry[1];
        }
        else  if( y <= 6.3 && y > 3.6){
            move.gantry[0] = 0;
            move.gantry[1] += offset_y;

            ROS_INFO_STREAM("Position of trg in move2trg y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  trg_10");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] += offset_final_y;
            goToPresetLocation(move_trg);
            return -move_trg.gantry[1];
        }
        else  if( y <= 3.6 && y > 0){
            offset_y = 0.6;
            offset_final_x = 0.85;
            move.gantry[0] = 0;
            ROS_INFO_STREAM("Position of trg in move2trg y (0):" << move.gantry[1]);
//            move.gantry[1] -= offset_y;

            ROS_INFO_STREAM("Position of trg in move2trg y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  trg_11");
            ROS_INFO_STREAM("Position of value of offset_y :" << offset_y);

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
//            move_trg.gantry[1] -= offset_y;
            goToPresetLocation(move_trg);
            return -move_trg.gantry[1];
        }
        else  if( y <= 0 && y > -3.6){
            move.gantry[0] = 0;
            move.gantry[1] += offset_y;

            ROS_INFO_STREAM("Position of trg in move2trg y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  trg_12");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] += offset_final_y;
            goToPresetLocation(move_trg);
            return -move_trg.gantry[1];
        }
        else  if( y <= -3.6 && y > -6.6){
            move.gantry[0] = 0;
            move.gantry[1] -= offset_y;

            ROS_INFO_STREAM("Position of trg in move2trg y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  trg_13");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] -= offset_final_y;
            goToPresetLocation(move_trg);
            return -move_trg.gantry[1];
        }
        else  if( y <= -6.6 ){
            move.gantry[0] = 0;
            move.gantry[1] += offset_y;

            ROS_INFO_STREAM("Position of trg in move2trg y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  trg_14");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] += offset_final_y;
            goToPresetLocation(move_trg);
            return -move_trg.gantry[1];
        }
        return 0;
    }
}

void GantryControl::pickFromConveyor(const Product &product, ConveyerParts &conveyerPartsObj) {
    ROS_INFO_STREAM("Going to pick " << product.type << " from conveyor ...");
    geometry_msgs::Pose estimated_conveyor_pose = product.estimated_conveyor_pose;
    ROS_DEBUG_STREAM("Estimated_conveyor_pose: " << estimated_conveyor_pose.position.x << std::endl
                                                 << estimated_conveyor_pose.position.y << std::endl
                                                 << estimated_conveyor_pose.position.z << std::endl
                                                 << estimated_conveyor_pose.orientation.x << std::endl
                                                 << estimated_conveyor_pose.orientation.y << std::endl
                                                 << estimated_conveyor_pose.orientation.z << std::endl
                                                 << estimated_conveyor_pose.orientation.w);
    
    conveyor_up_.gantry = {estimated_conveyor_pose.position.x, -estimated_conveyor_pose.position.y+0.2, 1.57};
    goToPresetLocation(conveyor_up_);

    ROS_INFO_STREAM("Waiting to pick up ... ");
    activateGripper("left_arm");
    auto left_gripper_status = getGripperState("left_arm");
    geometry_msgs::Pose pickup_pose, pre_pickup_pose;
    
    while(!left_gripper_status.enabled){
    	activateGripper("left_arm");
    	left_gripper_status = getGripperState("left_arm");
    }

	pickup_pose.position.x = estimated_conveyor_pose.position.x;
	pickup_pose.position.y = estimated_conveyor_pose.position.y;
	pickup_pose.position.z = estimated_conveyor_pose.position.z + model_height.at(product.type) + GRIPPER_HEIGHT;

	auto currentPose = left_arm_group_.getCurrentPose().pose;
	pickup_pose.orientation.x = currentPose.orientation.x;
	pickup_pose.orientation.y = currentPose.orientation.y;
	pickup_pose.orientation.z = currentPose.orientation.z;
	pickup_pose.orientation.w = currentPose.orientation.w;

    pre_pickup_pose = pickup_pose;
    pre_pickup_pose.position.z += 0.03; 

    left_arm_group_.setPoseTarget(pre_pickup_pose);
    left_arm_group_.move();  // Move to the pre pick up location
    left_arm_group_.setPoseTarget(pickup_pose);

    left_gripper_status = getGripperState("left_arm");

    while (!left_gripper_status.attached) {
        if(conveyerPartsObj.checkForPick()) {
            ROS_DEBUG_STREAM("Trying to pick it up");
            left_arm_group_.move();
            break;
        }
    }
    
    while (!left_gripper_status.attached) {
        ROS_DEBUG_STREAM_THROTTLE(10, "Waiting for part to be picked");
        left_gripper_status = getGripperState("left_arm");
    }

	ROS_INFO_STREAM("[Gripper] = object attached");
	//--Move arm to previous position
	left_arm_group_.setPoseTarget(pre_pickup_pose);
    left_arm_group_.move();
    left_arm_group_.setPoseTarget(currentPose);
    left_arm_group_.move();
    return;
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

/// Turn on vacuum gripper
void GantryControl::activateGripper(std::string arm_name) {
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = true;

    if (arm_name == "left_arm") {
        left_gripper_control_client.call(srv);
    } else {
        right_gripper_control_client.call(srv);
    }
    ROS_INFO_STREAM("[GantryControl][activateGripper] DEBUG: srv.response =" << srv.response);
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

