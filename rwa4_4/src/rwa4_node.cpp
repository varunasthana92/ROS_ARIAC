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
    //   "/ariac/logical_camera_1",
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
    logical_cam_subscribers.resize(logical_camera_topics.size());
    for(int i=0; i<logical_camera_topics.size(); i++) {
    logical_cam_subscribers[i] = node.subscribe<nist_gear::LogicalCameraImage>( logical_camera_topics[i], 10, 
                                                                          boost::bind(&BuildClass::logical_camera_callback,
                                                                                      &buildObj, _1, i+1));
    }
   
    // ros::Duration timeout(5.0);

    // geometry_msgs::TransformStamped C_to_W_transform;
    // tf2_ros::Buffer tfBuffer_;
    // try {
	// 	C_to_W_transform = tfBuffer_.lookupTransform("world", "logical_camera_1_frame", ros::Time(0), timeout);
	// }
    // catch (tf2::TransformException &ex) {
    //     ROS_FATAL_STREAM( "Nogt able to find the conveyer camera frame -- " << ex.what());
    //     ros::Duration(1.0).sleep();
	// }

    Competition comp(node);
    comp.init();

    std::string c_state = comp.getCompetitionState();
    comp.getClock();

    ros::Subscriber order_sub = node.subscribe("/ariac/orders", 1000, &BuildClass::orderCallback, &buildObj);
    
    ConveyerParts conve(node);
    GantryControl gantry(node);
    ros::Subscriber quality_sensor_1_sub = node.subscribe("/ariac/quality_control_sensor_1", 1000, &GantryControl::qualityCallback, &gantry);
    ros::Subscriber logical_camera_17_sub = node.subscribe("/ariac/ariac/logical_camera_17", 1000, &GantryControl::logicalCallback, &gantry);
    gantry.init();
    gantry.goToPresetLocation(gantry.start_);

    std::string arm = "left";
    for(auto &shipment: buildObj.order_recieved.shipments) {
        for(auto &product: shipment.products) {
            do {
                if(product.type.size()!=0) {
                    buildObj.queryPart(product);
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
                }
                else {
                    ROS_INFO_STREAM("No name of part");
                }
                }while(!gantry.placePart(product.p, shipment.agv_id, arm, node));
            // gantry.gantryCome(gantry.preLoc[product.p.camFrame]);
        }
        gantry.goToPresetLocation(gantry.start_);
        comp.shipAgv(shipment.agv_id, shipment.shipment_type);
    }


    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}