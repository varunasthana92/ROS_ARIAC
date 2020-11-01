#include "orderBuild.h"
#include <unordered_map>
#include "conveyer.h"

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

void BuildClass::setList(Product &product_received, int num_shipment, std::string shipment_type){
    ROS_DEBUG_STREAM("Setting part  " << product_received.type);
    if(queryPart(product_received)){
        ROS_INFO_STREAM("in static lsit part  " << product_received.type);
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
        if(num_shipment > ship_top_prod_static.size()){
            ship_top_prod_static.push_back(st_order);
        }else{
            ship_top_prod_static[num_shipment -1] = st_order;
        }
        
    }else{
        ROS_INFO_STREAM("in moving lsit part  " << product_received.type);
        product_received.mv_prod = true;
        if(mv_order){
            struct all_Order *temp = new(all_Order);
            temp->prod = mv_order->prod;
            temp->ship_num = mv_order->ship_num;
            temp->shipment_type = mv_order->shipment_type;
            temp->next = mv_order->next;

            mv_order->prod = product_received;
            mv_order->ship_num = num_shipment;
            mv_order->shipment_type = shipment_type;
            mv_order->next = temp;
        }else{
            mv_order = new(all_Order);
            mv_order->prod = product_received;
            mv_order->ship_num = num_shipment;
            mv_order->shipment_type = shipment_type;
            mv_order->next = NULL;
            mv_order_left = true;
        }
        if(num_shipment > ship_top_prod_moving.size()){
            ship_top_prod_moving.push_back(mv_order);
        }else{
            ship_top_prod_moving[num_shipment -1] = mv_order;
        }
    }
    return;    
}

struct all_Order* BuildClass::getList(ConveyerParts &conveyerPartsObj){
    int st_order_shipment_num_top = -1;
    int mv_order_shipment_num_top = -1;
    if(mv_order){
        ROS_DEBUG_STREAM("For moving lsit  " << mv_order->prod.type);
        mv_order_shipment_num_top = mv_order->ship_num;
    }
    
    if(st_order){
        ROS_DEBUG_STREAM("For static list  " << st_order->prod.type);
        st_order_shipment_num_top = st_order->ship_num;
    }

    struct all_Order* mv_temp = NULL;
    struct all_Order* st_temp = NULL;

    if(st_order_shipment_num_top != -1 && mv_order_shipment_num_top != -1){
        st_temp = st_order;
        mv_temp = mv_order;
        if(mv_temp->ship_num >= st_temp->ship_num){
            curr_build_shipment_num = mv_temp->ship_num;
            struct all_Order* mv_dummy_head = new(all_Order);
            mv_dummy_head->ship_num = -2;
            mv_dummy_head->next = mv_order;
            struct all_Order* mv_temp_prev = mv_dummy_head;
            do{
                bool status = conveyerPartsObj.giveClosestPart(mv_temp->prod.type, mv_temp->prod.estimated_conveyor_pose);
                // bool status = false;
                if(status){
                    ROS_INFO_STREAM("MOVING Part " << mv_temp->prod.type);
                    mv_temp_prev->next = mv_temp->next;
                    mv_order = mv_dummy_head->next;
                    delete(mv_dummy_head);
                    ROS_INFO_STREAM("MOVING Part " << mv_temp->prod.type);
                    return mv_temp;
                }
                mv_temp_prev = mv_temp;
                mv_temp = mv_temp->next;
            }while(mv_temp && mv_temp->ship_num == curr_build_shipment_num);
        }
        curr_build_shipment_num = st_temp->ship_num;
        st_order = st_order->next;
        st_temp->next = NULL;
        ROS_INFO_STREAM("Static Part " << st_temp->prod.type);
        return st_temp;

    }else if(st_order_shipment_num_top != -1){
        st_temp = st_order;
        curr_build_shipment_num = st_temp->ship_num;
        st_order = st_order->next;
        st_temp->next = NULL;
        ROS_INFO_STREAM("Static Part " << st_temp->prod.type);
        return st_temp;

    }else if(mv_order_shipment_num_top != -1){
        mv_temp = mv_order;
        curr_build_shipment_num = mv_temp->ship_num;

        struct all_Order* mv_dummy_head = new(all_Order);
        mv_dummy_head->ship_num = -2;
        mv_dummy_head->next = mv_order;
        struct all_Order* mv_temp_prev = mv_dummy_head;

        bool status = false;
        while(!status){
            status = conveyerPartsObj.giveClosestPart(mv_temp->prod.type, mv_temp->prod.estimated_conveyor_pose);
            if(status){
                ROS_INFO_STREAM("MOVING Part " << mv_temp->prod.type);
                mv_temp_prev->next = mv_temp->next;
                mv_order = mv_dummy_head->next;
                delete(mv_dummy_head);
                ROS_INFO_STREAM("MOVING Part " << mv_temp->prod.type);
                return mv_temp;
            }
            mv_temp_prev = mv_temp;
            mv_temp = mv_temp->next;
            if(mv_temp == NULL){
                mv_temp_prev = mv_dummy_head;
                mv_temp = mv_order;
            }
        }
    }

    return NULL;
}

void BuildClass::orderCallback(const nist_gear::Order& ordermsg) {
    Product product_recieved;
    Shipment shipment_recieved;
    Order order_recieved;
    order_recieved.order_id = ordermsg.order_id;
    for(const auto &ship: ordermsg.shipments) {
        num_shipment++;
        curr_build_shipment_num = num_shipment;
        shipment_recieved.shipment_type = ship.shipment_type;
        shipment_recieved.agv_id = ship.agv_id;
        for(const auto &prod: ship.products) {
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
        std::cout<< "\n wwwwwwwwwwwwwwwwww CAM ID = " <<cam_id <<"\n";
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