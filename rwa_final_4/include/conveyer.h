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

class Detection {
  public:
    bool part_set = false;
    bool picked_up = false;
    double first_look_time=-1;
    double second_look_time=-1;
    double speed=-1;
    std::string type="";
    geometry_msgs::Pose first_pose;
    geometry_msgs::Pose second_pose;
    geometry_msgs::Pose current_pose;
    void emptyDetection();
};

class ConveyerParts {
  public:
    ros::Subscriber conveyer_subscriber;
    Detection current_detection;
    std::vector<Detection> allConveyerParts;
    void conveyerLogicalCameraCallback(const nist_gear::LogicalCameraImage& msg);
    ConveyerParts(ros::NodeHandle& node); // Constructor
    void Init();
    bool checkPart(const std::string &part_name);
    double estimated_time=0;
    bool giveClosestPart(const std::string &part_name, geometry_msgs::Pose &poseOnConveyer);  // Will give the current location of the part on conveyer 
    bool checkForPick();
    void updateCurrentPoses();
    void checkBoundaries();
    void checkCurrentPartSet();

  private:
    double part_read_limit=2.5;
    double conveyer_end_y=-4;
    double offset = 1.8;
    double max_y_limit = conveyer_end_y+offset;
    geometry_msgs::Pose pick_pose;
    Detection pick_part;
    bool ready_for_pick=false;
    geometry_msgs::Pose getPose_W(const geometry_msgs::Pose &pose_C);
    ros::NodeHandle node_;
    // geometry_msgs::Pose current_pose;
    double start_time = 0;
    double velocity = 0;
    tf2_ros::Buffer tfBuffer_;
    void addNewPartToMap(Detection detection); 
    geometry_msgs::TransformStamped C_to_W_transform;
    void calculateSpeed();

};

#endif