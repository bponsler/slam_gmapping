/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 * 
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

#include <rclcpp/rclcpp.hpp>

#include "slam_gmapping.h"

class SlamGMappingNodelet : public rclcpp::node::Node
{
  public:
    SlamGMappingNodelet()
      :
      rclcpp::node::Node("gmapping")
    {
    }

    ~SlamGMappingNodelet() {}
  
    virtual void onInit()
    {
      //NODELET_INFO_STREAM("Initialising Slam GMapping nodelet...");
      printf("Initialising Slam GMapping nodelet...\n");
      sg_.reset(new SlamGMapping(rclcpp::node::Node::SharedPtr(this), rclcpp::node::Node::SharedPtr(this)));
      //NODELET_INFO_STREAM("Starting live SLAM...");
      printf("Starting live SLAM...\n");
      sg_->startLiveSlam();
    }

  private:  
    boost::shared_ptr<SlamGMapping> sg_;
};

#include <class_loader/class_loader_register_macro.h>

CLASS_LOADER_REGISTER_CLASS(SlamGMappingNodelet, rclcpp::Node)
