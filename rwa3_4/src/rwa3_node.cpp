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
#include <string>
#include <unordered_map>

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
#include <geometry_msgs/Pose.h>
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


struct similarParts{
    Part* parts_data;
    struct similarParts* next;
};

class allStaticParts{
private:
    std::unordered_map<std::string, similarParts* > map;
public:
    int getPart(Product prod){
        std::string name = prod.type;
        if(map.find(name) != map.end()){
            similarParts* temp = map[name];
            if(temp != NULL){
                map[name] = temp->next;
            }
            Part* data = temp->parts_data;
            delete(temp);
            prod.p = *data;
            return 1;
        }else{
            return 0;
        }
    }

    void setPart(similarParts* data){
        std::string name = data->parts_data->type;
        if(map.find(name) != map.end()){
            data->next = map[name];
        }
        map[name] = data;
        return;
    }
};


class BuildClass{
private:
    std::string pick;
    allStaticParts non_moving_part_data;
    bool callBackOnce[16];   // for 16 logical cameras, not including onveyor belt camera cam_id = 1

public:

    BuildClass(){
        for(int i = 0; i < 16; ++i){
            callBackOnce[i] = true;
        }
    }

    int queryPart(Product prod){
        return non_moving_part_data.getPart(prod);
    }
    void logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg, int cam_id){
        std::cout <<" logial cam id: " << cam_id << "\n";
        if(cam_id == 1)
            return;
        
        if( callBackOnce[cam_id-2]){
            ros::Duration timeout(5.0);
            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener(tfBuffer);
            int i=1, part_idx=1;

            while (i < msg->models.size()){
                std::string partName = msg->models[i].type;
                if (i!=1 && msg->models[i].type != msg->models[i-1].type) {
                    part_idx=1;
                }

                Part* detected_part = new(Part);
                detected_part->type = partName;
                detected_part->pose = msg->models[i].pose;
                detected_part->id = std::to_string(part_idx);
                detected_part->state = FREE;        

                std::string frame_name = "logical_camera_" + std::to_string(cam_id) + "_" + msg->models[i].type + "_" + std::to_string(part_idx) + "_frame";
                
                detected_part->frame = "logical_camera_" + std::to_string(cam_id);
                i++;
                part_idx++;
                geometry_msgs::TransformStamped transformStamped;
                transformStamped = tfBuffer.lookupTransform("world", frame_name, ros::Time(0), timeout);
                // tf2::Quaternion q(  transformStamped.transform.rotation.x,
                //                     transformStamped.transform.rotation.y,
                //                     transformStamped.transform.rotation.z,
                //                     transformStamped.transform.rotation.w);
                // tf2::Matrix3x3 m(q);
                // double roll, pitch, yaw;
                // m.getRPY(roll, pitch, yaw);
                detected_part->time_stamp = ros::Time(0);
                detected_part->save_pose.position.x = transformStamped.transform.translation.x;
                detected_part->save_pose.position.y = transformStamped.transform.translation.y;
                detected_part->save_pose.position.z = transformStamped.transform.translation.z;
                detected_part->save_pose.orientation = transformStamped.transform.rotation;

                similarParts* data = new(similarParts);
                data->parts_data = detected_part;
                data->next = NULL;
                non_moving_part_data.setPart(data);
            }
            callBackOnce[cam_id - 2] = false;
        }
        return;
    }
};


int main(int argc, char ** argv) {
    ros::init(argc, argv, "rwa3_node");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(50);
    spinner.start();

    std::vector<std::string> logical_camera_topics {
      "/ariac/logical_camera_1",
      "/ariac/logical_camera_2",
      "/ariac/logical_camera_3",
      "/ariac/logical_camera_4",
      "/ariac/logical_camera_5",
      "/ariac/logical_camera_6",
      "/ariac/logical_camera_7",
      "/ariac/logical_camera_8",
      "/ariac/logical_camera_9",
      "/ariac/logical_camera_10",
      "/ariac/logical_camera_11",
      "/ariac/logical_camera_12",
      "/ariac/logical_camera_13",
      "/ariac/logical_camera_14",
      "/ariac/logical_camera_15",
      "/ariac/logical_camera_16",
      "/ariac/logical_camera_17"
      };
    std::vector<ros::Subscriber> logical_cam_subscribers;
    int i=0;
    BuildClass buildObj;
    logical_cam_subscribers.resize(17);
    for(int i=0; i<17; i++) {
    logical_cam_subscribers[i] = node.subscribe<nist_gear::LogicalCameraImage>( logical_camera_topics[i], 10, 
                                                                          boost::bind(&BuildClass::logical_camera_callback,
                                                                                      &buildObj, _1, i+1));
    }

    
    Competition comp(node);
    comp.init();

    std::string c_state = comp.getCompetitionState();
    comp.getClock();

    ros::Subscriber order_sub = node.subscribe("/ariac/orders", 1000, orderCallback);


    GantryControl gantry(node);
    gantry.init();
    gantry.goToPresetLocation(gantry.start_);

    // ros::spinOnce();
    // int readPart = buildObj.non_moving_part_data.getPart("disk_part_blue");
    // if(readPart){
    //     std::cout << " PArt read \n";
    //     std::cout << readPart->type;
    //     return 0;
    // }
    // std::cout<<"not read parts\n";
    // return 0;

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