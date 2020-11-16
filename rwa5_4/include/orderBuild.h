#ifndef ORDERBUILD_H
#define ORDERBUILD_H

#include <ros/ros.h>
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include "utils.h"
#include "conveyer.h"
#include <utility>

struct all_Order{
    Product prod;
    int ship_num;
    std::string shipment_type;
    bool priority = false;
    struct all_Order *next;
};

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
    allStaticParts non_moving_conveyor_part_data;
    std::vector<int> temp;
    bool mv_order_left = false;
    bool st_order_left = false;
    std::vector<struct all_Order*> ship_top_prod_static;
    std::vector<struct all_Order*> ship_top_prod_moving;
    bool callBackOnce[16];   // for 16 logical cameras, not including onveyor belt camera cam_id = 1
    int camCount=0;
public:
    // struct agvInfo agv1, agv2;
    std::vector<Order> allOrders;
    struct all_Order *st_order = NULL;
    struct all_Order *mv_order = NULL;
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
    struct all_Order* getList(ConveyerParts &conveyerPartsObj);
    void pushList(struct all_Order* prod);   
    // struct all_Order* getList();
    int queryPart(Product &prod);    // pass by reference
    void logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg, int cam_id);
    void shelf_distance();
};

#endif