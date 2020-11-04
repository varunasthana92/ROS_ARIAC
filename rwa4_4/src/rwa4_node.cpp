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
#include "conveyer.h"

#include <tf2/LinearMath/Quaternion.h>

#include <ros/console.h>




int main(int argc, char ** argv) {
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
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
      // "/ariac/logical_camera_16",
      // "/ariac/logical_camera_17"
      };
    std::vector<ros::Subscriber> logical_cam_subscribers;

    BuildClass buildObj;
    logical_cam_subscribers.resize(logical_camera_topics.size());
    for(int i=0; i<logical_camera_topics.size(); i++) {
    logical_cam_subscribers[i] = node.subscribe<nist_gear::LogicalCameraImage>( logical_camera_topics[i], 10, 
                                                                          boost::bind(&BuildClass::logical_camera_callback,
                                                                                      &buildObj, _1, i+1));
    }

    Competition comp(node);
    comp.init();

    std::string c_state = comp.getCompetitionState();
    comp.getClock();

    ros::Subscriber order_sub = node.subscribe("/ariac/orders", 1000, &BuildClass::orderCallback, &buildObj);
    
    ConveyerParts conveyerPartsObj(node);
    GantryControl gantry(node);
    ros::Subscriber quality_sensor_1_sub = node.subscribe("/ariac/quality_control_sensor_1", 1000, &GantryControl::qualityCallback1, &gantry);
    ros::Subscriber logical_camera_17_sub = node.subscribe("/ariac/logical_camera_17", 1000, &GantryControl::logicalCallback17, &gantry);
    
    ros::Subscriber quality_sensor_2_sub = node.subscribe("/ariac/quality_control_sensor_2", 1000, &GantryControl::qualityCallback2, &gantry);
    ros::Subscriber logical_camera_16_sub = node.subscribe("/ariac/logical_camera_16", 1000, &GantryControl::logicalCallback16, &gantry);
    
    gantry.init();
    gantry.goToPresetLocation(gantry.start_);
       
    std::string arm = "left";
    int curr_build_shipment_num = -1;
    std::string curr_agv;
    std::string curr_shipment_type;
    struct all_Order* curr_prod = new(all_Order);

    while(buildObj.st_order || buildObj.mv_order){
        curr_prod = buildObj.getList(conveyerPartsObj);
        ROS_DEBUG_STREAM("For shipement " << curr_prod->ship_num);
        if(curr_build_shipment_num == -1){
            curr_build_shipment_num = curr_prod->ship_num;
            curr_agv = curr_prod->prod.agv_id;
            curr_shipment_type = curr_prod->shipment_type;

        }else if(curr_build_shipment_num != curr_prod->ship_num){
            if(curr_build_shipment_num > curr_prod->ship_num){
                gantry.goToPresetLocation(gantry.start_);
                comp.shipAgv(curr_agv, curr_shipment_type);
            }
            curr_build_shipment_num = curr_prod->ship_num;
            curr_agv = curr_prod->prod.agv_id;
            curr_shipment_type = curr_prod->shipment_type;
        }

        ROS_INFO_STREAM("For shipement " << curr_prod->ship_num);
        arm = "left";
        
        Product product = curr_prod->prod;
        ROS_INFO_STREAM("For product " << product.type << " cam " << product.p.camFrame);
        ROS_INFO_STREAM("For part " << product.p.type << " cam " << product.p.camFrame);

        if (product.mv_prod) {
            gantry.pickFromConveyor(product, conveyerPartsObj);
            product.p.rpy_init = quaternionToEuler(product.estimated_conveyor_pose);
        } else {
            ROS_DEBUG_STREAM("Not picking from conveyor!!!!!!!!!!!!");
            float Y_pose = gantry.move2trg(product.p.pose.position.x, -product.p.pose.position.y );
            gantry.pickPart(product.p);
            gantry.move2start(product.p.pose.position.x - 0.4, -Y_pose);
        }
        
        // product.p.pose = product.pose;
        if (product.pose.orientation.x == 1 || product.pose.orientation.x == -1) {
            gantry.flipPart();
            arm = "right";
            gantry.activateGripper("right_arm");
            gantry.deactivateGripper("left_arm");
        }

        bool status = true;
        if(product.agv_id == "agv1")
            status = gantry.placePart(product, product.agv_id, arm, buildObj.agv1);
        else
            status = gantry.placePart(product, product.agv_id, arm, buildObj.agv2);
        
        if(!status){
            buildObj.pushList(curr_prod);
        }else{
            delete(curr_prod);
        }
    }

    gantry.goToPresetLocation(gantry.start_);
    comp.shipAgv(curr_agv, curr_shipment_type);
    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}