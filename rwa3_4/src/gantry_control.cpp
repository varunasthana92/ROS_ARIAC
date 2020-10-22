#include "gantry_control.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nist_gear/LogicalCameraImage.h>

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

void GantryControl::rotate_gantry(double angle) {
    trajectory_msgs::JointTrajectory command_msg;
    trajectory_msgs::JointTrajectoryPoint points;
    points.positions = current_gantry_controller_state_.actual.positions;
    points.positions[1] = angle;
    command_msg.joint_names.push_back("small_long_joint");
    command_msg.joint_names.push_back("torso_base_main_joint");
    command_msg.joint_names.push_back("torso_rail_joint");
    command_msg.points.push_back(points);
    send_command(command_msg);
}

void GantryControl::setPrelocations() {
    preLoc[0] = start_;
    preLoc[1] = cam1_;
    preLoc[2] = cam2_;
    preLoc[3] = cam3_;
    preLoc[4] = cam4_;
    preLoc[5] = cam5_;
    preLoc[6] = cam6_;
    preLoc[7] = cam7_;
    preLoc[8] = cam8_;
    preLoc[9] = cam9_;
    preLoc[10] = cam10_;
    preLoc[11] = cam11_;
    preLoc[12] = cam12_;
    preLoc[13] = cam13_;
    preLoc[14] = cam14_;
    preLoc[15] = cam15_;
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

    agv1_.gantry = {-0.6, 6.9, 0.};
    agv1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv2_.gantry = {0.6, 6.9, PI};
    agv2_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    // cam1_.gantry = {3.104, 1.80, 0.};
    // cam1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    // cam1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
    
    cam2_.gantry = {3.0208, -1.7029, 0.};
    cam2_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    cam2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    cam3_.gantry = {4.9927, -1.7029, 0.};
    cam3_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    cam3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
    // Cam 4
    cam4_.gantry = {5.1227, 1.7322, 0.};
    cam4_.left_arm = {0, -PI/4, PI/2, -PI/4, PI/2, 0};
    cam4_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    cam5_.gantry = {3.104, 1.80, 0.};
    cam5_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    cam5_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    cam6_.gantry = {-15.77, 1.5, 0.};
    cam6_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    cam6_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    cam7_.gantry = {-13.77, 1.5, 0.};
    cam7_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    cam7_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
    
    cam8_.gantry = {-15.77, -4.20, 0.};
    cam8_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    cam8_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    cam9_.gantry = {-13.77, -4.20, 0.};
    cam9_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    cam9_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    cam10_.gantry = {-15.77, 4.3, 0.};
    cam10_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    cam10_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    cam11_.gantry = {-13.77, 4.3, 0.};
    cam11_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    cam11_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    cam12_.gantry = {4.93, 4.75, 0.};
    cam12_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    cam12_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    cam13_.gantry = {2.9, 4.75, 0.};
    cam13_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    cam13_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
    
    cam14_.gantry = {4.9, -4.7, 0.};
    cam14_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    cam14_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    cam15_.gantry = {2.85, -4.7, 0.};
    cam15_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    cam15_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
    setPrelocations();
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
                                                      std::string agv){
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    std::string kit_tray;
    if (agv.compare("agv1")==0)
        kit_tray = "kit_tray_1";
    else
        kit_tray = "kit_tray_2";
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "kit_tray_2";
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
            ee_target_tf = tfBuffer.lookupTransform("target_frame", "left_ee_link",
                                                 ros::Time(0), timeout);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }

    geometry_msgs::Pose world_target{target};
    world_target.position.x = world_target_tf.transform.translation.x;
    world_target.position.y = world_target_tf.transform.translation.y;
    world_target.position.z = world_target_tf.transform.translation.z;
    world_target.orientation.x = ee_target_tf.transform.rotation.x;
    world_target.orientation.y = ee_target_tf.transform.rotation.y;
    world_target.orientation.z = ee_target_tf.transform.rotation.z;
    world_target.orientation.w = ee_target_tf.transform.rotation.w;

    return world_target;
}

bool GantryControl::pickPart(part part){
    //--Activate gripper
    activateGripper("left_arm");
//    ros::AsyncSpinner spinner(1);
//    spinner.start();

//    left_arm_group_.setPoseReferenceFrame("world");
    geometry_msgs::Pose currentPose = left_arm_group_.getCurrentPose().pose;

//    ROS_INFO_STREAM("[left_arm_group_]= " << currentPose.position.x << ", " << currentPose.position.y << "," << currentPose.position.z);

    part.pose.position.z = part.pose.position.z + model_height.at(part.type) + GRIPPER_HEIGHT - EPSILON;
    part.pose.orientation.x = currentPose.orientation.x;
    part.pose.orientation.y = currentPose.orientation.y;
    part.pose.orientation.z = currentPose.orientation.z;
    part.pose.orientation.w = currentPose.orientation.w;
//    ROS_INFO_STREAM("["<< part.type<<"]= " << part.pose.position.x << ", " << part.pose.position.y << "," << part.pose.position.z << "," << part.pose.orientation.x << "," << part.pose.orientation.y << "," << part.pose.orientation.z << "," << part.pose.orientation.w);


    auto state = getGripperState("left_arm");
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

// Variable to store if current part is faulty
bool is_part_faulty = false;

// Variable to hold faulty part pose
geometry_msgs::Pose faulty_part_pose;

// Quality control sensor 1 callback
void qualityCallback(const nist_gear::LogicalCameraImage& msg) {
    if (msg.models.size() != 0) {
        ROS_INFO_STREAM("Detected faulty part!: " << (msg.models[0]).type);
        is_part_faulty = true;
        geometry_msgs::Pose model_pose = (msg.models[0]).pose;
        /*
        ROS_INFO_STREAM("Faulty part pose: " 
                    << model_pose.position.x << std::endl
                    << model_pose.position.y << std::endl 
                    << model_pose.position.z << std::endl
                    << model_pose.orientation.x << std::endl
                    << model_pose.orientation.y << std::endl
                    << model_pose.orientation.z << std::endl
                    << model_pose.orientation.w);*/

    // Transform pose detected from quality sensor 1 to world frame
    /* [TODO] Need to refactor this part as seperate function to tranform pose in one
     reference frame to another (For reusability) */
    geometry_msgs::TransformStamped transformStamped;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Duration timeout(3.0);
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
    faulty_part_pose = new_pose.pose;
    
    ROS_INFO_STREAM("Transformed order part pose detected from quality sensor: " 
                    << new_pose.pose.position.x << std::endl
                    << new_pose.pose.position.y << std::endl
                    << new_pose.pose.position.z << std::endl
                    << new_pose.pose.orientation.x << std::endl
                    << new_pose.pose.orientation.y << std::endl
                    << new_pose.pose.orientation.z << std::endl
                    << new_pose.pose.orientation.w);
    
    }
}


void GantryControl::placePart(part part, 
                              std::string agv, 
                              ros::NodeHandle node){
    ros::Subscriber quality_sensor_1_sub = node.subscribe("/ariac/quality_control_sensor_1", 1000, qualityCallback);
    auto target_pose_in_tray = getTargetWorldPose(part.pose, agv);
    ROS_INFO_STREAM("Settled tray pose:" << target_pose_in_tray.position.x << " " 
                                         << target_pose_in_tray.position.y << " "
                                         << target_pose_in_tray.position.z);
    ros::Duration(3.0).sleep();
    goToPresetLocation(agv2_);
    ROS_INFO_STREAM("Trying to roate gantry");
    rotate_gantry(4.8);
    target_pose_in_tray.position.z += (ABOVE_TARGET + 1.5*model_height[part.type]);

    left_arm_group_.setPoseTarget(target_pose_in_tray);
    left_arm_group_.move();
    deactivateGripper("left_arm");
    auto state = getGripperState("left_arm");
    
    ros::Duration(2).sleep();
    if (state.attached) {
        std::cout << "Part faulty: " << is_part_faulty << std::endl;
        if (is_part_faulty) {
            std::cout << "Part faulty inside: " << is_part_faulty << std::endl;
            part.pose = faulty_part_pose;
            pickPart(part);
        }
        goToPresetLocation(start_);
        deactivateGripper("left_arm");
    }
}

void GantryControl::gantryGo(PresetLocation location) {
    double x,y,a;
    x = location.gantry[0];
    y = location.gantry[1];
    a = location.gantry[2];
    location.gantry[0] = 0;
    location.gantry[2] = 0;
    goToPresetLocation(location);
    location.gantry[0] = x;
    goToPresetLocation(location);
    location.gantry[2] = a;
    goToPresetLocation(location);
}

void GantryControl::gantryCome(PresetLocation location) {
    location.gantry[0] = 0;
    goToPresetLocation(location);
    location.gantry[1] = 0;
    goToPresetLocation(location);
    location.gantry[2] = 0;
    goToPresetLocation(location);
    auto state = getGripperState("left_arm");
    if (state.attached)
        deactivateGripper("left_arm");
}

bool GantryControl::move2start ( float x, float y ) {

    // geometry_msgs::Pose robot_pose = getRobotPose();
    // float x = curr_pose.position.x;
    // float y = curr_pose.position.y;
    ROS_INFO_STREAM("Position of trg in move2start y:" << y);
    ROS_INFO_STREAM("Position of trg in move2start robot x:" << x);
    ROS_INFO_STREAM("Position of trg in move2start robot x:" << y);
    float offset_y = 1.3;
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
            move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, 0, 0, 0};

            ROS_INFO_STREAM("Position of trg in move2start y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  str_7");

            goToPresetLocation(move);

            move.gantry[0] = 0;
            move.gantry[1] -= offset_y;
            move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, 0, 0, 0};
            goToPresetLocation(move);
            goToPresetLocation(start_);
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
            move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, 0, 0, 0};

            ROS_INFO_STREAM("Position of trg in move2start y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  str_7");

            goToPresetLocation(move);

            move.gantry[0] = 0;
            move.gantry[1] -= offset_y;
            move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, 0, 0, 0};
            goToPresetLocation(move);
            goToPresetLocation(start_);
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
            move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, 0, 0, 0};

            ROS_INFO_STREAM("Position of trg in move2start y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  str_7");

            goToPresetLocation(move);

            move.gantry[0] = 0;
            move.gantry[1] -= offset_y;
            move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, 0, 0, 0};
            goToPresetLocation(move);
            goToPresetLocation(start_);
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
            ROS_INFO_STREAM("Position of trg  str_111");

            goToPresetLocation(move);
            goToPresetLocation(start_);
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
            move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, 0, 0, 0};
            ROS_INFO_STREAM("Position of trg in move2trg y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  trg_7");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] -= offset_final_y;
            move_trg.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, 0, 0, 0};
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
            move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, 0, 0, 0};
            ROS_INFO_STREAM("Position of trg in move2trg y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  trg_5");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] -= offset_final_y;
            move_trg.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, 0, 0, 0};
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
            move.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, 0, 0, 0};
            ROS_INFO_STREAM("Position of trg in move2trg y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  trg_7");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] -= offset_final_y;
            move_trg.left_arm = {-PI/2, -PI/2, PI/2 + PI/4, 0, 0, 0};
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
            move.gantry[0] = 0;
            move.gantry[1] -= offset_y;

            ROS_INFO_STREAM("Position of trg in move2trg y:" << move.gantry[1]);
            ROS_INFO_STREAM("Position of trg  trg_11");

            goToPresetLocation(move);
            
            move_trg.gantry[0] -= offset_final_x;
            move_trg.gantry[1] -= offset_final_y;
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

