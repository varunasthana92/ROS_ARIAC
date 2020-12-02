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
 * @brief Similar parts struct
 */
struct similarParts{
    Part* parts_data;
    struct similarParts* next;
};
/**
 * @brief allStaticParts Class
 */
class allStaticParts{
private:
    std::unordered_map<std::string, similarParts* > map;
public:
    int getPart(Product &prod); //pass by reference

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
    /**
     * @brief Clear AGV1
     */
    bool clear_agv1 = false;
    /**
     * @brief Clear AGV2
     */
    bool clear_agv2 = false;
    /**
     * @brief Clear AGV1 for shipment type
     */
    std::string clear_agv1_for_ship_type = "";
    /**
     * @brief Clear AGV2 for shipment type
     */
    std::string clear_agv2_for_ship_type = "";
    /**
     * @brief Most recent order agv1
     */
    std::vector<int> most_recent_order_agv1;
    /**
     * @brief Most recent order agv2
     */
    std::vector<int> most_recent_order_agv2;
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