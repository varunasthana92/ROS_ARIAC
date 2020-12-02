/**
 *  Copyright 2020 Varun Asthana, Saumil Shah, Nalin Das, Markose Jacob, Aditya Goswami
 *  @file rwa_final_node.cpp
 * 
 *  @author Varun Asthana
 *  @author Saumil Shah
 *  @author Nalin Das
 *  @author Markose Jacob
 *  @author Aditya Goswami
 * 
 *  @date 11/29/2020
 *  @brief Main file for rwa5
 * 
 *  @section DESCRIPTION
 *
 *  Source main file for rwa5
 *
 */

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

void clockCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}



int main(int argc, char ** argv) {
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::init(argc, argv, "rwa_final_node");
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

    ros::Subscriber order_sub = node.subscribe("/ariac/orders", 2, &BuildClass::orderCallback, &buildObj);
    
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

    while(buildObj.order_read == false){
        continue;
    }

    int num_obstacles = obstObj.num_obstacles;

    while(buildObj.st_order || buildObj.mv_order){
        curr_prod = buildObj.getList(conveyerPartsObj, num_obstacles);

        if(curr_prod->prod.agv_id == "agv1"){
            buildObj.agv1_allocated = true;
        }else{
            buildObj.agv2_allocated = true;
        }

        ROS_DEBUG_STREAM("For shipement " << curr_prod->shipment_type);
        ROS_DEBUG_STREAM("For shipement num " << curr_prod->ship_num);
        ROS_DEBUG_STREAM("On agv " << curr_prod->prod.agv_id);

        if(buildObj.clear_agv1){
            if( buildObj.clear_agv1_for_ship_type == curr_prod->shipment_type){
                gantry.clearAgv(curr_prod->prod.agv_id, buildObj);
                buildObj.clear_agv1 = false;
                buildObj.clear_agv1_for_ship_type = "";
            }
        }

        if(buildObj.clear_agv2){
            if( buildObj.clear_agv2_for_ship_type == curr_prod->shipment_type){
                gantry.clearAgv("agv2", buildObj);
                buildObj.clear_agv2 = false;
                buildObj.clear_agv2_for_ship_type = "";
            }
        }

        curr_build_shipment_num = curr_prod->ship_num;
        curr_agv = curr_prod->prod.agv_id;
        curr_shipment_type = curr_prod->shipment_type;
        if(comp.agv_ship_data.find(curr_shipment_type) == comp.agv_ship_data.end()){
            comp.agv_ship_data[curr_shipment_type] = curr_agv;
        }

        arm = "left";
        Product product = curr_prod->prod;
        ROS_INFO_STREAM("To pick " << product.type << " cam " << product.p.camFrame);
        bool pickstatus = false;
        if (product.mv_prod) {
            pickstatus = gantry.pickFromConveyor(product, conveyerPartsObj);
            if(pickstatus == false){
                buildObj.ship_build_count[curr_prod->ship_num]++;
                continue;
            }
            product.p.rpy_init = quaternionToEuler(product.estimated_conveyor_pose);
        } else {
            float gantryX = 0.4;
            float gantryY = 0;
            int currGap = -1;
            std::vector<double> left_arm = { 0, 0, 0, 0, 0, 0};
            std::vector<double> right_arm = gantry.start_.right_arm;
            bool ready2pick = obstObj.isAisleClear(product.p.aisle_num);
            product.p.obstacle_free = ready2pick;
            ROS_WARN_STREAM("Main() Obstacle Free? : "<< ready2pick);
            pickstatus = false;
            if(! ready2pick){
                gantry.move2closestGap(product.p, buildObj.positionGap, buildObj.gapNum, 1, gantryX,
                                                        gantryY, obstObj, currGap, left_arm);
                ROS_DEBUG_STREAM("Main() Aisle of Interest: "<< product.p.aisle_num);
                while(!pickstatus){
                    // for aisles with obstacles: trigger to pick part
                    do{
                        ready2pick = obstObj.moveBot(product.p.pose.position.x, -3, product.p.aisle_num, gantryX, currGap);
                    }while(! ready2pick);
                    right_arm = { PI, 0, 0, 0, 0, 0};
                    left_arm = gantry.move2trg(product.p.pose.position.x, -product.p.pose.position.y, gantryX, gantryY, currGap, left_arm, right_arm);
                    pickstatus = gantry.pickPart(product.p);
                    gantry.escape(product.p.aisle_num, buildObj.positionGap, buildObj.gapNum, 1, gantryX, gantryY,
                                  obstObj, currGap, left_arm, pickstatus);
                }
            }else{
                left_arm = gantry.start_.left_arm;
                right_arm = gantry.start_.right_arm;
                left_arm = gantry.move2trg(product.p.pose.position.x, -product.p.pose.position.y, gantryX, gantryY, currGap, left_arm);
                pickstatus = gantry.pickPart(product.p);
            }
            gantry.move2start(gantryX, -gantryY, left_arm, right_arm);
        }

        if (std::abs(product.pose.orientation.x) <= 1 && std::abs(product.pose.orientation.x) >= 0.98)  {
            gantry.flipPart();
            product.p.flip_part = true;
            arm = "right";
            product.rpy_final = quaternionToEuler(product.pose);
            product.rpy_final[0] = 0;
            tf2::Quaternion q_deliver(product.rpy_final[2], product.rpy_final[1], product.rpy_final[0]);
            product.pose.orientation.x = q_deliver.x();
            product.pose.orientation.y = q_deliver.y();
            product.pose.orientation.z = q_deliver.z();
            product.pose.orientation.w = q_deliver.w();

            tf2::Quaternion q_pi( 0, 0, 1, 0);
            tf2::Quaternion q_pi_by_2( 0, 0, 0.7071068, 0.7071068);
            tf2::Quaternion q_init_part(product.p.pose.orientation.x,
                                product.p.pose.orientation.y,
                                product.p.pose.orientation.z,
                                product.p.pose.orientation.w);
            tf2::Quaternion q_rslt = q_init_part*q_pi_by_2.inverse();

            product.p.pose.orientation.x = q_rslt.x();
            product.p.pose.orientation.y = q_rslt.y();
            product.p.pose.orientation.z = q_rslt.z();
            product.p.pose.orientation.w = q_rslt.w();
        }

        product.p.obstacle_free = true; //to try pick pick again and again if faulty
        bool status = true;
        status = gantry.placePart(product, product.agv_id, arm, curr_prod, conveyerPartsObj);
        if(!status){
            ROS_WARN_STREAM("Main() Part place FAIL ");
            buildObj.pushList(curr_prod);
        }else{
            ros::Duration(0.5).sleep();
            buildObj.ship_build_count[curr_prod->ship_num]++;
            // ROS_WARN_STREAM("Main() Part place SUCCESS ");
            if(buildObj.ship_build_count[curr_prod->ship_num] == buildObj.num_prod_in_ship[curr_prod->ship_num -1 ]){
                comp.agv_ship_data.erase(curr_shipment_type);
                if(curr_agv == "agv1"){
                    gantry.goToPresetLocation(gantry.agv1_drop);
                    comp.shipAgv(curr_agv, curr_shipment_type);
                    buildObj.most_recent_order_agv1.pop_back();
                    buildObj.agv1_allocated = false;
                    gantry.agv1_allParts.prod_on_tray.clear();
                    gantry.agv1_allParts.count = 0;
                    for(int i = 0; i < gantry.agv1_allParts.complete_order_data.size(); ++i){
                        delete(gantry.agv1_allParts.complete_order_data[i]);
                    }
                    gantry.agv1_allParts.complete_order_data.clear();
                }else{
                    gantry.goToPresetLocation(gantry.agv2_drop);
                    comp.shipAgv(curr_agv, curr_shipment_type);
                    buildObj.most_recent_order_agv2.pop_back();
                    buildObj.agv2_allocated = false;
                    gantry.agv2_allParts.prod_on_tray.clear();
                    gantry.agv2_allParts.count = 0;
                    for(int i = 0; i < gantry.agv2_allParts.complete_order_data.size(); ++i){
                        delete(gantry.agv2_allParts.complete_order_data[i]);
                    }
                    gantry.agv2_allParts.complete_order_data.clear();
                }
            }
        }
    }
    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}