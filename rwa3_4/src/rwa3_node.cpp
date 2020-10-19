// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <algorithm>
#include <vector>

#include <ros/ros.h>

#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //--needed for tf2::Matrix3x3

#include "competition.h"
#include "utils.h"
#include "gantry_control.h"

#include <tf2/LinearMath/Quaternion.h>

void orderCallback(const nist_gear::Order& ordermsg) {
    Order order_recieved;
    Product product_recieved;
    Shipment shipment_recieved;
    order_recieved.order_id = ordermsg.order_id;
    for(const auto &ship: ordermsg.shipments) {
        shipment_recieved.shipment_type = ship.shipment_type;
        shipment_recieved.agv_id = ship.agv_id;
        for(const auto &prod: ship.products) {
            product_recieved.type = prod.type;
            product_recieved.pose = prod.pose;
            shipment_recieved.products.emplace_back(product_recieved);
        }
        order_recieved.shipments.push_back(shipment_recieved);
    }
    ROS_INFO_STREAM("I heard: " << order_recieved.order_id);
    for(auto s: order_recieved.shipments) {
        ROS_INFO_STREAM("Order type: " << s.shipment_type);
    }
}

class Build{
private:
    part part2pick;

public:

    void logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg, int cam_id){
        ros::Duration timeout(5.0);
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        int i=1, part_idx=1;
        while (i < msg->models.size()){
          if (i!=1 && msg->models[i].type != msg->models[i-1].type) {
            part_idx=1;
          }
          std::string frame_name = "logical_camera_" + std::to_string(cam_id) + "_" + msg->models[i].type + "_" + std::to_string(part_idx) + "_frame";
          i++;
          part_idx++;
          geometry_msgs::TransformStamped transformStamped;
          transformStamped = tfBuffer.lookupTransform("world", frame_name, ros::Time(0), timeout);
          tf2::Quaternion q(
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z,
            transformStamped.transform.rotation.w);
          tf2::Matrix3x3 m(q);
          double roll, pitch, yaw;
          m.getRPY(roll, pitch, yaw);
        
        ROS_INFO("%s in world frame : [%.2f,%.2f,%.2f] [%.2f,%.2f,%.2f]", frame_name.c_str(), transformStamped.transform.translation.x,
          transformStamped.transform.translation.y,
          transformStamped.transform.translation.z,
          roll,
          pitch,
          yaw);
        }
        ROS_INFO_STREAM("------------------------------------------");
      }
}


int main(int argc, char ** argv) {
    ros::init(argc, argv, "rwa3_node");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(50);
    spinner.start();

    Competition comp(node);
    comp.init();

    std::string c_state = comp.getCompetitionState();
    comp.getClock();

    ros::Subscriber order_sub = node.subscribe("/ariac/orders", 1000, orderCallback);

    GantryControl gantry(node);
    gantry.init();
    gantry.goToPresetLocation(gantry.start_);

    //--1-Read order
    //--2-Look for parts in this order
    //--We go to this bin because a camera above
    //--this bin found one of the parts in the order
    gantry.goToPresetLocation(gantry.bin3_);


    //--You should receive the following information from a camera
    part my_part;
    my_part.type = "pulley_part_red";
    my_part.pose.position.x = 4.365789;
    my_part.pose.position.y = 1.173381;
    my_part.pose.position.z = 0.728011;
    my_part.pose.orientation.x = 0.012;
    my_part.pose.orientation.y = -0.004;
    my_part.pose.orientation.z = 0.002;
    my_part.pose.orientation.w = 1.000;

    //--get pose of part in tray from /ariac/orders
    part part_in_tray;
    part_in_tray.type = "pulley_part_red";
    part_in_tray.pose.position.x = -0.12;
    part_in_tray.pose.position.x = -0.2;
    part_in_tray.pose.position.x = 0.0;
    part_in_tray.pose.orientation.x = 0.0;
    part_in_tray.pose.orientation.y = 0.0;
    part_in_tray.pose.orientation.z = 0.0;
    part_in_tray.pose.orientation.w = 1.0;

    //--Go pick the part
    gantry.pickPart(my_part);
    //--Go place the part
    gantry.placePart(part_in_tray, "agv2");

    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}