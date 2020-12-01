#ifndef ORDERBUILD_H
#define ORDERBUILD_H

#include <ros/ros.h>
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include "utils.h"
#include "conveyer.h"
#include <utility>

struct similarParts{
    Part* parts_data;
    struct similarParts* next;
};

class allStaticParts{
private:
    std::unordered_map<std::string, similarParts* > map;
public:
    int getPart(Product &prod); //pass by reference

    void setPart(similarParts* data);
};


class BuildClass{
private:
    std::string pick;
    int num_shipment = 0;
    int curr_build_shipment_num;
    Part *conveyor_Part = NULL;
    allStaticParts non_moving_part_data;
    std::vector<int> temp;
    bool mv_order_left = false;
    bool st_order_left = false;
    bool callBackOnce[16];   // for 16 logical cameras, not including onveyor belt camera cam_id = 1
    int camCount=0;
    
public:
    // struct agvInfo agv1, agv2;
    bool agv1_allocated = false;
    bool agv2_allocated = false;
    bool order_read = false;
    std::vector<Order> allOrders;
    std::vector<int> num_prod_in_ship; // store the count of items in each shipment while reading the order
    std::unordered_map<int, int> ship_build_count; // update the map<ship_id, items on agv> while building the order
    struct all_Order *st_order = NULL;
    struct all_Order *mv_order = NULL;
    bool clear_agv1 = false;
    bool clear_agv2 = false;
    std::string clear_agv1_for_ship_type = "";
    std::string clear_agv2_for_ship_type = "";
    std::vector<int> most_recent_order_agv1;
    std::vector<int> most_recent_order_agv2;
public:
    BuildClass(){
        for(int i = 0; i < 16; ++i){
            callBackOnce[i] = true;
        }
    }
    std::vector< std::pair<float , float> > positionGap;
    std::vector<int> gapNum;
    void orderCallback(const nist_gear::Order& ordermsg);
    void setList(Product &product_received, int num_shipment, std::string shipment_type);
    struct all_Order* getList(ConveyerParts &conveyerPartsObj, int num_obstacles);
    void pushList(struct all_Order* prod);   
    // struct all_Order* getList();
    int queryPart(Product &prod);    // pass by reference
    void logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg, int cam_id);
    void shelf_distance();
};

#endif