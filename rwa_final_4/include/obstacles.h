#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <nist_gear/Proximity.h>
#include <unordered_map>

// const double PI = 3.141592; // TODO correct!



// node.subscribe<nist_gear::Proximity>(breakbeam_sensor_topics[i], 10, \
//             boost::bind(&GantryControl::breakbeam_sensor_callback, &gantry, _1, i+1));

class ObstaclesInAisle {
  private:
    
    std::vector<ros::Subscriber> breakbeam_subscriber_1;
    std::vector<ros::Subscriber> breakbeam_subscriber_2;
    std::vector<ros::Subscriber> breakbeam_subscriber_3;
    std::vector<ros::Subscriber> breakbeam_subscriber_4;

    std::vector<std::string> breakbeam_sensor_topics_1{
      "/ariac/breakbeam_10",
      "/ariac/breakbeam_11",
      "/ariac/breakbeam_12",
      "/ariac/breakbeam_13",
      "/ariac/breakbeam_14",
      "/ariac/breakbeam_15"
      };
    std::vector<std::string> breakbeam_sensor_topics_2{
      "/ariac/breakbeam_20",
      "/ariac/breakbeam_21",
      "/ariac/breakbeam_22",
      "/ariac/breakbeam_23",
      "/ariac/breakbeam_24",
      "/ariac/breakbeam_25"
      };
    std::vector<std::string> breakbeam_sensor_topics_3{
      "/ariac/breakbeam_30",
      "/ariac/breakbeam_31",
      "/ariac/breakbeam_32",
      "/ariac/breakbeam_33",
      "/ariac/breakbeam_34",
      "/ariac/breakbeam_35"
      };
    std::vector<std::string> breakbeam_sensor_topics_4{
      "/ariac/breakbeam_40",
      "/ariac/breakbeam_41",
      "/ariac/breakbeam_42",
      "/ariac/breakbeam_43",
      "/ariac/breakbeam_44",
      "/ariac/breakbeam_45"
      };
  public:
    int num_obstacles = 0;
    std::vector<int> aisle_1_sensor_data;
    std::vector<int> aisle_2_sensor_data;
    std::vector<int> aisle_3_sensor_data;
    std::vector<int> aisle_4_sensor_data;
    
    std::pair<int, int> aisle_1_dir;
    std::pair<int, int> aisle_2_dir;
    std::pair<int, int> aisle_3_dir;
    std::pair<int, int> aisle_4_dir;

    std::vector<bool> aisle_clear;
    
    ros::NodeHandle node_;
    // std::unordered_map<int, double> last_seen_time;
    void breakbeam_callback(const nist_gear::Proximity::ConstPtr &msg, int id);
    bool moveBot(float destX, int gapNum, int aisle_num, float currX, int currGap);
    ObstaclesInAisle(ros::NodeHandle& node); // Constructor
    bool isAisleClear(int aisle_num);
    void Init();
  // private:
};

#endif