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
    ros::Subscriber quality_sensor_1_sub = node.subscribe("/ariac/quality_control_sensor_1", 1000, &GantryControl::qualityCallback, &gantry);
    ros::Subscriber logical_camera_17_sub = node.subscribe("/ariac/ariac/logical_camera_17", 1000, &GantryControl::logicalCallback, &gantry);
    gantry.init();
    gantry.goToPresetLocation(gantry.start_);

    // ros::spinOnce();

    //--1-Read order
    //--2-Look for parts in this order
    //--We go to this bin because a camera above
    //--this bin found one of the parts in the order
    
    std::string arm = "left";
    for(int oid =0; oid < buildObj.allOrders.size(); ++oid){
            // auto &allshipment = buildObj.allOrders[oid].shipments;
        for(auto &shipment : buildObj.allOrders[oid].shipments){
            shipment.parent_order = &buildObj.allOrders[oid];
            shipment.parent_order_idx = oid;
            if(shipment.prodComplete == shipment.products.size()){
                continue;
            }
            for(auto &product: shipment.products){
                int status = 0;
                do {
                    product.parent_shipment = &shipment;
                    if(!buildObj.queryPart(product)){
                        shipment.products.push_back(product);
                        break;
                    }
                    ROS_INFO_STREAM("For product " << product.tray << " cam " << product.p.camFrame);
                    // gantry.conveyor();
                    // ROS_INFO_STREAM("Preloc size " << gantry.preLoc.size());
                    ROS_INFO_STREAM("Position of trg y:" << product.p.pose.position.y);
                    float Y_pose = gantry.move2trg(product.p.pose.position.x, -product.p.pose.position.y );
                    // gantry.gantryGo(gantry.preLoc[product.p.camFrame]);
                    gantry.pickPart(product.p);

                    //geometry_msgs::Pose robot_pose = gantry.getRobotPose();
                    gantry.move2start(product.p.pose.position.x - 0.4, -Y_pose);
                    // gantry.gantryCome(gantry.preLoc[product.p.camFrame]);
                    product.p.pose=product.pose;
                    if (product.p.pose.orientation.x == 1 || product.p.pose.orientation.x == -1) {
                        std::cout << "product.p.pose.orientation.x inside: " << product.p.pose.orientation.x << std::endl;
                        gantry.flipPart();
                        arm = "right";
                        gantry.activateGripper("right_arm");
                        gantry.deactivateGripper("left_arm");
                    }
                    if(shipment.agv_id == "agv1")
                        status = gantry.placePart(product, shipment.agv_id, arm, buildObj.agv1);
                    else
                        status = gantry.placePart(product, shipment.agv_id, arm, buildObj.agv2);

                    if(status)
                        shipment.prodComplete++;
                }while(!status);
            }
            gantry.goToPresetLocation(gantry.start_);
            comp.shipAgv(shipment.agv_id, shipment.shipment_type);
        }
    }

    // for(int i =0; i < buildObj.order_recieved.shipments.size(); ++i){
    //     for(int j =0; j < buildObj.order_recieved.shipments[i].products.size(); ++j){
    //         ProdGantryControl
    //         PresetLocation tempPose = gantry.cam4_;
    //         tempPose.gantry[0] = my_part.pose.position.x - 0.4;
    //         tempPose.gantry[1] = -my_part.pose.position.y;

    //         gantry.goToPresetLocation(tempPose);
    //         part part_in_tray;
    //         part_in_tray.pose = currProd.pose;

    //         gantry.pickPart(my_part);

    //         std::string agv_to_build =  buildObj.order_recieved.shipments[i].agv_id;
    //         gantry.placePart(buildObj.order_recieved.shipments[i].products[j].p, "agv2");
    //         int temp;
    //         std::cin >> temp;
    //     }
    // }
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

    // // --Go pick the part
    // gantry.pickPart(my_part);
    // // --Go place the part
    // gantry.placePart(part_in_tray, "agv2");

    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}