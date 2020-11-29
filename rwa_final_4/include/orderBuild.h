/**
 *  Copyright 2020 Varun Asthana, Saumil Shah, Nalin Das, Markose Jacob, Aditya Goswami
 *  @file orderBuild.h
 * 
 *  @author Varun Asthana
 *  @author Saumil Shah
 *  @author Nalin Das
 *  @author Markose Jacob
 *  @author Aditya Goswami
 * 
 *  @date 11/29/2020
 * 
 *  @brief BuildClass and allStaticParts class
 * 
 *  @section DESCRIPTION
 *
 *  Header file for the BuildClass and allStaticParts class
 *
 */
#ifndef ORDERBUILD_H
#define ORDERBUILD_H

#include <ros/ros.h>
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include "utils.h"
#include "conveyer.h"
#include <utility>

/**
 * @brief All order struct
 */
struct all_Order{
    Product prod;
    int ship_num;
    std::string shipment_type;
    bool priority = false;
    struct all_Order *next;
};
/**
 * @brief Similar parts struct
 */
struct similarParts{
    Part* parts_data;
    struct similarParts* next;
};
/**
 * @brief allStaticParts class
 */
class allStaticParts{
private:
    /**
     * @brief Hashmap to store similar parts
     */
    std::unordered_map<std::string, similarParts* > map;
public:
    /**
     * @brief Gets part from product
     * @param prod Product
     * @return 1 if product has part else 0
     */
    int getPart(Product &prod); 
    /**
     * @brief Set part
     * @param data Similar parts data
     * @return None
     */
    void setPart(similarParts* data);
};

/**
 * @brief Build class
 */
class BuildClass{
private:
    /**
     * @brief Pick variable
     */
    std::string pick;
    /**
     * @brief Numbers of shipments
     */
    int num_shipment = 0;
    /**
     * @brief Current build shipment number
     */
    int curr_build_shipment_num;
    /**
     * @brief Conveyor Part
     */
    Part *conveyor_Part = NULL;
    /**
     * @brief Non moving part data
     */
    allStaticParts non_moving_part_data;
    /**
     * @brief Non moving conveyor part data
     */
    allStaticParts non_moving_conveyor_part_data;
    /**
     * @brief Temp array
     */
    std::vector<int> temp;
    /**
     * @brief Mv order left
     */
    bool mv_order_left = false;
    /**
     * @brief St order left
     */
    bool st_order_left = false;
    /**
     * @brief Ship top product static
     */
    std::vector<struct all_Order*> ship_top_prod_static;
    /**
     * @brief Ship top product moving
     */
    std::vector<struct all_Order*> ship_top_prod_moving;
    /**
     * @brief Array for 16 logical cameras
     */
    bool callBackOnce[16]; 
    /**
     * @brief Camera Count
     */
    int camCount=0;

public:
    /**
     * @brief AGV1 Allocated
     */
    bool agv1_allocated = false;
    /**
     * @brief AGV2 Allocated
     */
    bool agv2_allocated = false;
    /**
     * @brief Order read
     */
    bool order_read = false;
    /**
     * @brief All Orders array
     */
    std::vector<Order> allOrders;
    /**
     * @brief Number of products in shipment
     */
    std::vector<int> num_prod_in_ship;
    /**
     * @brief Shipment build count
     */
    std::unordered_map<int, int> ship_build_count;
    /**
     * @brief St order
     */
    struct all_Order *st_order = NULL;
    /**
     * @brief Mv order
     */
    struct all_Order *mv_order = NULL;
public:
    /**
     * @brief Constructor
     * @param None None
     * @return None
     */
    BuildClass(){
        for(int i = 0; i < 16; ++i){
            callBackOnce[i] = true;
        }
    }
    /**
     * @brief Position Gap vector
     */
    std::vector< std::pair<float , float> > positionGap;
    /**
     * @brief Gap Number
     */
    std::vector<int> gapNum;
    /**
     * @brief Order Callback
     * @param ordermsg Order message
     * @return None
     */
    void orderCallback(const nist_gear::Order& ordermsg);
    /**
     * @brief Set List
     * @param product_received Product recieved 
     * @param num_shipment Shipment number
     * @param shipment_type Shipment type
     * @return None
     */
    void setList(Product &product_received, int num_shipment, std::string shipment_type);
    /**
     * @brief Get List function
     * @param conveyerPartsObj Conveyor Parts Object
     * @param num_obstacles Number of obstacles
     * @return All orders
     */
    struct all_Order* getList(ConveyerParts &conveyerPartsObj, int num_obstacles);
    /**
     * @brief Push List
     * @param prod Product
     * @return None
     */
    void pushList(struct all_Order* prod);   
    /**
     * @brief Query Part
     * @param prod Product
     * @return Part number
     */
    int queryPart(Product &prod);
    /**
     * @brief Logical camera callback
     * @param msg Logical Camera Image
     * @return None
     */
    void logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg, int cam_id);
    /**
     * @brief Shelf Distance
     * @param None None
     * @return None
     */
    void shelf_distance();
};

#endif