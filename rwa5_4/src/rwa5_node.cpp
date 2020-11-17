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
#include "obstacles.h"

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
      "/ariac/logical_camera_15"
      // "/ariac/logical_camera_16",
      // "/ariac/logical_camera_17"
      };
    std::vector<ros::Subscriber> logical_cam_subscribers;

    BuildClass buildObj;
    logical_cam_subscribers.resize(logical_camera_topics.size());
    buildObj.shelf_distance();
    for(int i=0; i<logical_camera_topics.size(); i++) {
        logical_cam_subscribers[i] = node.subscribe<nist_gear::LogicalCameraImage>( logical_camera_topics[i], 10, 
                                                                          boost::bind(&BuildClass::logical_camera_callback,
                                                                                      &buildObj, _1, i+1));
    }

    ObstaclesInAisle obstObj(node);

    Competition comp(node);
    comp.init();

    std::string c_state = comp.getCompetitionState();
    comp.getClock();

    ros::Subscriber order_sub = node.subscribe("/ariac/orders", 1000, &BuildClass::orderCallback, &buildObj);
    
    ConveyerParts conveyerPartsObj(node);
    GantryControl gantry(node);

    ros::Subscriber quality_sensor_1_sub = node.subscribe("/ariac/quality_control_sensor_1", 1, &GantryControl::qualityCallback2, &gantry);
    ros::Subscriber logical_camera_17_sub = node.subscribe("/ariac/logical_camera_17", 1, &GantryControl::logicalCallback17, &gantry);
    
    ros::Subscriber quality_sensor_2_sub = node.subscribe("/ariac/quality_control_sensor_2", 1, &GantryControl::qualityCallback1, &gantry);
    ros::Subscriber logical_camera_16_sub = node.subscribe("/ariac/logical_camera_16", 1, &GantryControl::logicalCallback16, &gantry);
    
    gantry.init();
    gantry.goToPresetLocation(gantry.start_);
       
    std::string arm = "left";
    int curr_build_shipment_num = -1;
    std::string curr_agv;
    std::string curr_shipment_type;
    struct all_Order* curr_prod = new(all_Order);

    while(buildObj.st_order || buildObj.mv_order){
        curr_prod = buildObj.getList(conveyerPartsObj);
        ROS_DEBUG_STREAM("For shipement " << curr_prod->shipment_type);
        ROS_DEBUG_STREAM("For shipement num " << curr_prod->ship_num);
        ROS_DEBUG_STREAM("On agv " << curr_prod->prod.agv_id);

        curr_build_shipment_num = curr_prod->ship_num;
        curr_agv = curr_prod->prod.agv_id;
        curr_shipment_type = curr_prod->shipment_type;

        arm = "left";
        
        Product product = curr_prod->prod;
        ROS_INFO_STREAM("To pick " << product.type << " cam " << product.p.camFrame);

        if (product.mv_prod) {
            gantry.pickFromConveyor(product, conveyerPartsObj);
            product.p.rpy_init = quaternionToEuler(product.estimated_conveyor_pose);
        } else {
            float gantryX = 0;
            float gantryY = 0;
            int currGap = -1;
            std::vector<double> left_arm;
            bool pickstatus = false;
            bool ready2pick = obstObj.isAisleClear(product.p.aisle_num);
            product.p.obstacle_free = ready2pick;
            if(! ready2pick){
                ROS_DEBUG_STREAM("Aisle number for part " << product.p.aisle_num);
                gantry.move2closestGap(product.p, buildObj.positionGap, buildObj.gapNum, 1, gantryX,
                                                        gantryY, obstObj, currGap);
                ROS_WARN_STREAM("Curr Gap in main: " << currGap);
                while(!pickstatus){
                    // for aisles with obstacles: trigger to pick part
                    do{
                        ready2pick = obstObj.moveBot(product.p.pose.position.x, -3, product.p.aisle_num, gantryX, currGap);
                    }while(! ready2pick);

                    left_arm = gantry.move2trg(product.p.pose.position.x, -product.p.pose.position.y, gantryX, gantryY, currGap);
                    pickstatus = gantry.pickPart(product.p);
                    gantry.escape(product.p.aisle_num, buildObj.positionGap, buildObj.gapNum, 1, gantryX, gantryY,
                                  obstObj, currGap, left_arm, pickstatus);
                }
            }else{
                left_arm = gantry.move2trg(product.p.pose.position.x, -product.p.pose.position.y, gantryX, gantryY, currGap);
                pickstatus = gantry.pickPart(product.p);
            }
            product.p.obstacle_free = true;
            //escape plan
            ready2pick = obstObj.isAisleClear(product.p.aisle_num);
            if(! ready2pick){
                gantry.escape(product.p.aisle_num, buildObj.positionGap, buildObj.gapNum, 1, gantryX, gantryY,
                            obstObj, currGap, left_arm, pickstatus);

                ready2pick = obstObj.isAisleClear(product.p.aisle_num);
                PresetLocation temp = gantry.start_;
                if(gantryX != 0 && ready2pick){
                    if(product.p.aisle_num == 1){
                        temp.gantry[1] = -buildObj.positionGap[0].second;
                    }else {
                        temp.gantry[1] = -buildObj.positionGap[4].second;
                    }
                    temp.gantry[0] = gantryX;
                    temp.left_arm = { 0, 0, 0, 0, 0, 0};
                    temp.right_arm = { PI, 0, 0, 0, 0, 0};
                    gantry.goToPresetLocation(temp);
                    gantryX = temp.gantry[0];
                    gantryY = -temp.gantry[1];

                }else if(gantryX != 0 && !ready2pick){
                    bool move = false;
                    do{
                        move = obstObj.moveBot(0, -3, product.p.aisle_num, gantryX, currGap);

                    }while(!move);

                    if(product.p.aisle_num == 1){
                        temp.gantry[1] = -buildObj.positionGap[0].second;
                    }else {
                        temp.gantry[1] = -buildObj.positionGap[4].second;
                    }
                    temp.gantry[0] = gantryX;
                    temp.left_arm = { 0, 0, 0, 0, 0, 0};
                    temp.right_arm = { PI, 0, 0, 0, 0, 0};
                    gantry.goToPresetLocation(temp);
                    gantryX = temp.gantry[0];
                    gantryY = -temp.gantry[1];
                }

            }
            gantry.move2start(gantryX, -gantryY);            
        }
        
        // product.p.pose = product.pose;
        if (product.pose.orientation.x == 1 || product.pose.orientation.x == -1) {
            gantry.flipPart();
            arm = "right";
            product.p.rpy_init[0] = 0;
            tf2::Quaternion q_part(product.p.rpy_init[2], product.p.rpy_init[1], product.p.rpy_init[0]);
            product.p.pose.orientation.x = q_part.x();
            product.p.pose.orientation.y = q_part.y();
            product.p.pose.orientation.z = q_part.z();
            product.p.pose.orientation.w = q_part.w();
        }

        bool status = true;
        status = gantry.placePart(product, product.agv_id, arm);
        if(!status){
            buildObj.pushList(curr_prod);
        }else{
            buildObj.ship_build_count[curr_prod->ship_num]++;
            if(buildObj.ship_build_count[curr_prod->ship_num] == buildObj.num_prod_in_ship[curr_prod->ship_num]){
                gantry.goToPresetLocation(gantry.start_);
                comp.shipAgv(curr_agv, curr_shipment_type);
            }
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