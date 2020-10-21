#include "orderBuild.h"
#include <unordered_map>

int allStaticParts::getPart(Product &prod){
    std::string name = prod.type;
    // name = "pulley_part_green";
    if(map.find(name) != map.end()){
        similarParts* temp = map[name];
        Part* data = NULL;
        if(temp != NULL){
            map[name] = temp->next;
            data = temp->parts_data;     
            delete(temp);
            prod.p = *data;
            return 1;
        }
        // std::cout << "\nread from function " << prod.p.frame << std::endl;
        return 0;
    }else{
        return 0;       
    }
}

void allStaticParts::setPart(similarParts* data){
    std::string name = data->parts_data->type;
    if(map.find(name) != map.end()){
        data->next = map[name];
    }
    map[name] = data;
    return;
}

void BuildClass::orderCallback(const nist_gear::Order& ordermsg) {
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

int BuildClass::queryPart(Product &prod){
    return non_moving_part_data.getPart(prod);
}

void BuildClass::logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg, int cam_id){
    if(cam_id == 1)
        return;
    
    if( callBackOnce[cam_id-2]){
        ros::Duration timeout(5.0);
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        int i=0, part_idx=1;

        while (i < msg->models.size()){
            std::string partName = msg->models[i].type;
            if (i!=0 && msg->models[i].type != msg->models[i-1].type) {
                part_idx=1;
            }

            Part* detected_part = new(Part);
            detected_part->type = partName;
            detected_part->save_pose = msg->models[i].pose;
            detected_part->id = std::to_string(part_idx);
            detected_part->state = FREE;
            detected_part->camFrame = cam_id;        

            std::string frame_name = "logical_camera_" + std::to_string(cam_id) + "_" + msg->models[i].type + "_" + std::to_string(part_idx) + "_frame";
            // std::cout<< "\n" <<frame_name <<"\n";
            detected_part->frame = "logical_camera_" + std::to_string(cam_id);
            i++;
            part_idx++;
            geometry_msgs::TransformStamped transformStamped;
            try{

                transformStamped = tfBuffer.lookupTransform("world", frame_name, ros::Time(0), timeout);
                // tf2::Quaternion q(  transformStamped.transform.rotation.x,
                //                     transformStamped.transform.rotation.y,
                //                     transformStamped.transform.rotation.z,
                //                     transformStamped.transform.rotation.w);
                // tf2::Matrix3x3 m(q);
                // double roll, pitch, yaw;
                // m.getRPY(roll, pitch, yaw);
                detected_part->time_stamp = ros::Time(0);
                detected_part->pose.position.x = transformStamped.transform.translation.x;
                detected_part->pose.position.y = transformStamped.transform.translation.y;
                detected_part->pose.position.z = transformStamped.transform.translation.z;
                detected_part->pose.orientation = transformStamped.transform.rotation;

                similarParts* data = new(similarParts);
                data->parts_data = detected_part;
                data->next = NULL;
                non_moving_part_data.setPart(data);
            }
            catch (...) {
                continue;
            }
        }
        callBackOnce[cam_id - 2] = false;
    }
    return;
}