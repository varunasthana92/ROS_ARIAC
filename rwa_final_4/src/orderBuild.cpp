/**
 *  Copyright 2020 Varun Asthana, Saumil Shah, Nalin Das, Markose Jacob, Aditya Goswami
 *  @file orderBuild.cpp
 * 
 *  @author Varun Asthana
 *  @author Saumil Shah
 *  @author Nalin Das
 *  @author Markose Jacob
 *  @author Aditya Goswami
 * 
 *  @date 11/29/2020
 * 
 *  @brief BuildClass and allStaticParts class implementation
 * 
 *  @section DESCRIPTION
 *
 *  Source file for BuildClass and allStaticParts class methods
 *  implementation
 *
 */
#include "orderBuild.h"
#include <unordered_map>
#include "conveyer.h"
#include "utils.h"
#include <utility>

int allStaticParts::getPart(Product &prod){
    std::string name = prod.type;
    if(map.find(name) != map.end()){
        similarParts* temp = map[name];
        Part* data = NULL;
        if(temp != NULL){
            map[name] = temp->next;
            data = temp->parts_data;     
            delete(temp);
            prod.p = *data;
            prod.p.agv_id = prod.agv_id;
            return 1;
        }else{
            map.erase(name);
        }
        
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

void BuildClass::setList(Product &product_received, int num_shipment, std::string shipment_type){
    // ROS_DEBUG_STREAM("Setting part  " << product_received.type);
    if(queryPart(product_received)){
        ROS_DEBUG_STREAM("Setting in static list part  " << product_received.type);
        if(st_order){
            struct all_Order *temp = new(all_Order);
            temp->prod = st_order->prod;
            temp->ship_num = st_order->ship_num;
            temp->shipment_type = st_order->shipment_type;
            temp->next = st_order->next;

            st_order->prod = product_received;
            st_order->ship_num = num_shipment;
            st_order->shipment_type = shipment_type;
            st_order->next = temp;
        }else{
            st_order = new(all_Order);
            st_order->prod = product_received;
            st_order->ship_num = num_shipment;
            st_order->shipment_type = shipment_type;
            st_order->next = NULL;
            st_order_left = true;
        }
        
    }else{
        ROS_DEBUG_STREAM("Setting in moving list part  " << product_received.type);
        product_received.mv_prod = true;
        if(mv_order){
            struct all_Order *temp = new(all_Order);
            temp->prod = mv_order->prod;
            temp->ship_num = mv_order->ship_num;
            temp->shipment_type = mv_order->shipment_type;
            temp->next = mv_order->next;

            mv_order->prod = product_received;
            mv_order->prod.p.agv_id = product_received.agv_id;
            mv_order->ship_num = num_shipment;
            mv_order->shipment_type = shipment_type;
            mv_order->next = temp;
        }else{
            mv_order = new(all_Order);
            mv_order->prod = product_received;
            mv_order->prod.p.agv_id = product_received.agv_id;
            mv_order->ship_num = num_shipment;
            mv_order->shipment_type = shipment_type;
            mv_order->next = NULL;
            mv_order_left = true;
        }
    }
    return;    
}

void BuildClass::pushList(struct all_Order* prod){
    struct all_Order* temp;
    struct all_Order* prev;
    struct all_Order* dummy = new(all_Order);
    dummy->ship_num = -1;
    dummy->next = NULL;
    prev = dummy;

    bool st_found = 0;
    if(prod->prod.mv_prod){
        ROS_WARN_STREAM("PushList() in mv: ");
        dummy->next = mv_order;
        temp = mv_order;
        if(temp == NULL){
            mv_order = prod;
            mv_order->next = NULL;
            delete(dummy);
            return;
        }
    }else{
        st_found = 1;
        queryPart(prod->prod);
        ROS_WARN_STREAM("PushList() in st: ");
        dummy->next = st_order;
        temp = st_order;
        if(temp == NULL){
            st_order = prod;
            st_order->next = NULL;
            delete(dummy);
            return;
        }
    }

    ROS_WARN_STREAM("PushList() NOT empty");
    // ROS_WARN_STREAM("temp part " << temp->prod.type << " ship: " << temp->ship_num);
    // ROS_WARN_STREAM("added part " << prod->prod.type << " ship: " << prod->ship_num);
    while(temp->next && temp->ship_num > prod->ship_num){
        prev = temp;
        temp = temp->next;
    }

    prev->next = prod;
    prod->next = temp;
    if(st_found){
        st_order = dummy->next;
    }else{
        mv_order = dummy->next;
    }

    // next = temp->next;
    // temp->next = prod;
    // temp = temp->next;
    // temp->next = next;
    delete(dummy);
    return;
}

struct all_Order* BuildClass::getList(ConveyerParts &conveyerPartsObj, int num_obstacles){
    int st_order_shipment_num_top = -1;
    int mv_order_shipment_num_top = -1;

    if(mv_order){
        mv_order_shipment_num_top = mv_order->ship_num;
        ROS_WARN_STREAM("getList() mv_list left ");
    }
    
    if(st_order){
        st_order_shipment_num_top = st_order->ship_num;
        ROS_WARN_STREAM("getList() st_list left ");
    }
    if(most_recent_order_agv1.size()){
        ROS_WARN_STREAM("getList() agv1 top order " << most_recent_order_agv1.back());
    }
    
    if(most_recent_order_agv2.size()){
        ROS_WARN_STREAM("getList() agv2 top order " << most_recent_order_agv2.back());
    }
    
    struct all_Order* mv_temp = NULL;
    struct all_Order* st_temp = NULL;

    if(mv_order_shipment_num_top != -1){
        mv_temp = mv_order;
        std::string mv_temp_agv = mv_temp->prod.agv_id;
        // curr_build_shipment_num = mv_temp->ship_num;

        struct all_Order* mv_dummy_head = new(all_Order);
        mv_dummy_head->ship_num = -2;
        mv_dummy_head->next = mv_order;
        struct all_Order* mv_temp_prev = mv_dummy_head;
        int mv_temp_agv_max;

        if(mv_temp_agv == "agv1"){
            mv_temp_agv_max = most_recent_order_agv1.back();
        }else{
            mv_temp_agv_max = most_recent_order_agv2.back();
        }

        bool status = false;
        while(!status ){
            ROS_WARN_STREAM("getList() mv_temp agv " << mv_temp->prod.agv_id << " " << mv_temp->prod.type << " ship: " << mv_temp->ship_num);
             ROS_WARN_STREAM("getList() mv_temp_max " << mv_temp_agv_max);
            if(mv_temp->ship_num >= mv_temp_agv_max){
                status = conveyerPartsObj.giveClosestPart(mv_temp->prod.type, mv_temp->prod.estimated_conveyor_pose);
                if(status){
                    ROS_INFO_STREAM("Assigning MOVING Part " << mv_temp->prod.type);
                    mv_temp_prev->next = mv_temp->next;
                    mv_order = mv_dummy_head->next;
                    delete(mv_dummy_head);
                    // ROS_INFO_STREAM("MOVING Part " << mv_temp->prod.type);
                    mv_temp->prod.p.type = mv_temp->prod.type;
                    mv_temp->prod.p.camFrame = 1;
                    return mv_temp;
                }
            }
            mv_temp_prev = mv_temp;
            mv_temp = mv_temp->next;
            if(mv_temp == NULL){
                if(num_obstacles == 0 && st_order_shipment_num_top != -1){
                    break;
                }
                mv_temp_prev = mv_dummy_head;
                mv_temp = mv_temp_prev->next;
            }

            mv_temp_agv = mv_temp->prod.agv_id;
            if(mv_temp_agv == "agv1"){
                mv_temp_agv_max = most_recent_order_agv1.back();
            }else{
                mv_temp_agv_max = most_recent_order_agv2.back();
            }
        }
        delete(mv_dummy_head);

    }

    if(st_order_shipment_num_top != -1){
        st_temp = st_order;
        curr_build_shipment_num = st_temp->ship_num;
        st_order = st_order->next;
        st_temp->next = NULL;
        ROS_INFO_STREAM("Assigning STATIC Part " << st_temp->prod.type);
        return st_temp;
    }

    return NULL;
}

void BuildClass::orderCallback(const nist_gear::Order& ordermsg) {
    while(camCount < 14)
        continue;
    Shipment shipment_recieved;
    Order order_recieved;
    order_recieved.order_id = ordermsg.order_id;

    for(auto ship: ordermsg.shipments) {
        num_shipment++;
        curr_build_shipment_num = num_shipment;
        shipment_recieved.shipment_type = ship.shipment_type;
        if(ship.agv_id == "agv1"){
            most_recent_order_agv1.push_back(num_shipment);
            if(agv1_allocated == true){
                clear_agv1 = true;
                clear_agv1_for_ship_type  = ship.shipment_type;
            }
            agv1_allocated = true;
        }
        else if(ship.agv_id == "agv2"){
            most_recent_order_agv2.push_back(num_shipment);
            if(agv2_allocated == true){
                clear_agv2 = true;
                clear_agv2_for_ship_type  = ship.shipment_type;
            }
            agv2_allocated = true;
        }
        else{
            if(not agv1_allocated){
                most_recent_order_agv1.push_back(num_shipment);
                ship.agv_id = "agv1";
                agv1_allocated = true;
            }
            else if(not agv2_allocated){
                most_recent_order_agv2.push_back(num_shipment);
                ship.agv_id = "agv2";
                agv2_allocated = true;
            }
            else{
                most_recent_order_agv2.push_back(num_shipment);
                ship.agv_id = "agv2";
                agv2_allocated = true;
                clear_agv2 = true;
                clear_agv2_for_ship_type  = ship.shipment_type;
            }
        }
        shipment_recieved.agv_id = ship.agv_id;
        num_prod_in_ship.push_back(ship.products.size());
        for(const auto &prod: ship.products) {
            Product product_recieved;
            product_recieved.type = prod.type;
            product_recieved.agv_id = ship.agv_id;
            product_recieved.pose = prod.pose;
            // product_recieved.p.pose = prod.pose;
            product_recieved.shipId = num_shipment;
            shipment_recieved.products.emplace_back(product_recieved);
            setList(product_recieved, num_shipment, ship.shipment_type);
        }
        order_recieved.shipments.push_back(shipment_recieved);
    }
    ROS_INFO_STREAM("I heard: " << order_recieved.order_id);
    order_read = true;
    return;
}   

int BuildClass::queryPart(Product &prod){
    return non_moving_part_data.getPart(prod);
}

float shelfDis(const std::string &name1, const std::string & name2, float& leg_x){
    tf2_ros::Buffer tfBuffer;
    ros::Duration timeout(5.0);
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped shelfpose1, shelfpose2;
    try {
        shelfpose1 = tfBuffer.lookupTransform("world", name1, ros::Time(0), timeout);
        shelfpose2 = tfBuffer.lookupTransform("world", name2, ros::Time(0), timeout);
        leg_x = shelfpose2.transform.translation.x;
        return std::fabs(shelfpose1.transform.translation.x - shelfpose2.transform.translation.x);
    }
    catch(...){
        return 2000;
    }
}

float shelfDisCorner(const std::string &name1, float corner_x){
    tf2_ros::Buffer tfBuffer;
    ros::Duration timeout(5.0);
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped shelfpose1;
    float dis;
    try {
        shelfpose1 = tfBuffer.lookupTransform("world", name1, ros::Time(0), timeout);
        dis = (shelfpose1.transform.translation.x - corner_x);
        return std::abs(dis);
    }
    catch(...){
        return 2000;
    }
}

void BuildClass::shelf_distance(){
    std::string shelf[9];
    for(int i=3; i<=11; i++){
        shelf[i-3] = "shelf" + std::to_string(i) + "_frame";
    }
    float corner1 = -2.093;
    float corner2 = -16.7;
    std::vector<float> y_val = {3.0, 0,-3.0 };
    float gap_width_x = 2.176/2, gap_width_y = 1.2282/2 ;
    std::vector< std::pair<float , float> > gaps(5, {0,0});
    std::vector<int> gapNum_(5, -1);
    for(int i =0; i < 3; ++i){
        float disCorner = 0;
        do{
            disCorner = shelfDisCorner(shelf[i*3], corner1);
        }while(disCorner == 2000);
        if(disCorner > 0.2 && disCorner <= 2 * gap_width_x + 0.2){
            gaps[i+1] = {corner1 - gap_width_x, y_val[i]};
            gapNum_[i+1] = 1;
            continue;
        }

        disCorner = 0;
        do{
            disCorner = shelfDisCorner(shelf[i*3 + 2], corner2);
        }while(disCorner == 2000);

        if(disCorner >= 4.5 && disCorner <= 4.12 + 2 * gap_width_x + 0.2){
            gaps[i+1] = {corner2 + gap_width_x, y_val[i]};
            gapNum_[i+1] = 4;
            continue;
        }


        for(int sh=0; sh < 2; sh++){
            float dis = 0;
            float leg_x = 0;
            do{
                dis = shelfDis(shelf[i*3 + sh], shelf[i*3 + sh + 1], leg_x);
            }while(dis == 2000);

            if(dis > 4.2){
                gaps[i+1] = {leg_x + gap_width_x, y_val[i]};
                gapNum_[i+1] = (std::abs(leg_x) / 4.2) + 1;
                break;
            }
        }
    }
    positionGap = gaps;
    gapNum = gapNum_;
    return;
}

void BuildClass::logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg, int cam_id){
    if(cam_id == 1)
        return;
    
    
    if(callBackOnce[cam_id-2]){
        std::string frame_name = "logical_camera_" + std::to_string(cam_id) + "_frame";
        
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Duration timeout(5.0);
        geometry_msgs::TransformStamped transformStamped;
        
        int part_idx=1;
        for(int i = 0; i < msg->models.size(); ++i){
            std::string partName = msg->models[i].type;
            if (i!=0 && msg->models[i].type != msg->models[i-1].type) {
                part_idx=1;
            }

            bool found = false;

            while(!found) {
                try{
                    transformStamped = tfBuffer.lookupTransform("world", frame_name, ros::Time(0), timeout);                
                    // detected_part->time_stamp = ros::Time(0);
                }
                catch (tf2::TransformException &ex) {
                    ROS_FATAL_STREAM( "Not able to find the camera frame -- " << ex.what());
                }
                if(transformStamped.child_frame_id.size()>0) found=true;
            }

            Part* detected_part = new(Part);
            detected_part->type = partName;
            detected_part->save_pose = msg->models[i].pose;
            detected_part->id = std::to_string(part_idx);
            detected_part->state = FREE;
            detected_part->camFrame = cam_id;            
            detected_part->frame = frame_name;
            part_idx++;
            
            geometry_msgs::Pose world_pose;
            tf2::doTransform(detected_part->save_pose, world_pose, transformStamped);
            detected_part->pose = world_pose;

            if(detected_part->pose.position.x < 0){
                if(detected_part->pose.position.y >= 3.05 )
                    detected_part->aisle_num = 1;
                if(detected_part->pose.position.y >= 0 && detected_part->pose.position.y < 3.05)
                    detected_part->aisle_num = 2;
                if(detected_part->pose.position.y >= -3.05 && detected_part->pose.position.y < 0)
                    detected_part->aisle_num = 3;
                if(detected_part->pose.position.y < -3.05 )
                    detected_part->aisle_num = 4;
            }

            detected_part->rpy_init = quaternionToEuler(detected_part->pose);

            similarParts* data = new(similarParts);
            data->parts_data = detected_part;
            data->next = NULL;
            non_moving_part_data.setPart(data);
        }
        camCount++;
        callBackOnce[cam_id - 2] = false;
    }
    return;
}