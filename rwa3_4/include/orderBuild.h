#ifndef ORDERBUILD_H
#define ORDERBUILD_H

#include <ros/ros.h>
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include "utils.h"

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
    allStaticParts non_moving_part_data;
    bool callBackOnce[16];   // for 16 logical cameras, not including onveyor belt camera cam_id = 1
public:
    Order order_recieved;
public:
    BuildClass(){
        for(int i = 0; i < 16; ++i){
            callBackOnce[i] = true;
        }
    }
    void orderCallback(const nist_gear::Order& ordermsg);
    int queryPart(Product &prod);    // pass by reference
    void logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg, int cam_id);
};

#endif