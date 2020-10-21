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

#include "orderBuild.h"
#include "competition.h"
#include "utils.h"
#include "gantry_control.h"

#include <tf2/LinearMath/Quaternion.h>



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

    ros::Subscriber order_sub = node.subscribe("/ariac/orders", 1000, &BuildClass::orderCallback, &buildObj);


    GantryControl gantry(node);
    gantry.init();
    gantry.goToPresetLocation(gantry.start_);

    

    ros::spinOnce();
    std::cout<<" Part queried: " << buildObj.order_recieved.shipments[0].products[0].type << "\n";
    int readPart = buildObj.queryPart(buildObj.order_recieved.shipments[0].products[0]);
    if(readPart){
        std::cout << " first time Part read \n";
        std::cout << buildObj.order_recieved.shipments[0].products[0].p.frame <<std::endl;
        std::cout << buildObj.order_recieved.shipments[0].products[0].p.type <<std::endl;
        std::cout << buildObj.order_recieved.shipments[0].products[0].p.pose <<std::endl;
    }
    else{
        std::cout << " NOT read \n";
        return 0;
    }
    //--1-Read order
    //--2-Look for parts in this order
    //--We go to this bin because a camera above
    //--this bin found one of the parts in the order

    for(int i =0; i < buildObj.order_recieved.shipments.size(); ++i){

        for(int j =0; j < buildObj.order_recieved.shipments[i].products.size(); ++j){
            Product currProd = buildObj.order_recieved.shipments[i].products[j];
            buildObj.queryPart(currProd);
            Part my_part = currProd.p;
            // preset locations

            // auto check  = gantry.preLoc[my_part.camFrame];
            std::cout << " %%%%%%% check = " << my_part.camFrame << "\n";
            // gantry.goToPresetLocation(gantry.bin3_);

            // std::cout<< "\n x = "<< gantry.preLoc[my_part.camFrame].gantry[0]<<"\n";

            gantry.goToPresetLocation(gantry.cam4_);
            std::cout << " ###### check = " << my_part.camFrame << "\n";
            part part_in_tray;
            part_in_tray.pose = currProd.pose;

            gantry.pickPart(my_part);

            std::string agv_to_build =  buildObj.order_recieved.shipments[i].agv_id;
            gantry.placePart(part_in_tray, "agv2");
            std::cout<< "####### ENTER ########### ";
            int temp;
            std::cin >> temp;
        }
    }
    // gantry.goToPresetLocation(gantry.bin3_);


    // // --You should receive the following information from a camera
    // part my_part;
    // my_part.type = "pulley_part_red";
    // my_part.pose.position.x = 4.365789;
    // my_part.pose.position.y = 1.173381;
    // my_part.pose.position.z = 0.728011;
    // my_part.pose.orientation.x = 0.012;
    // my_part.pose.orientation.y = -0.004;
    // my_part.pose.orientation.z = 0.002;
    // my_part.pose.orientation.w = 1.000;

    // // --get pose of part in tray from /ariac/orders
    // part part_in_tray;
    // part_in_tray.type = "pulley_part_red";
    // part_in_tray.pose.position.x = -0.12;
    // part_in_tray.pose.position.x = -0.2;
    // part_in_tray.pose.position.x = 0.0;
    // part_in_tray.pose.orientation.x = 0.0;
    // part_in_tray.pose.orientation.y = 0.0;
    // part_in_tray.pose.orientation.z = 0.0;
    // part_in_tray.pose.orientation.w = 1.0;

    // --Go pick the part
    // gantry.pickPart(my_part);
    // --Go place the part
    // gantry.placePart(part_in_tray, "agv2");

    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}