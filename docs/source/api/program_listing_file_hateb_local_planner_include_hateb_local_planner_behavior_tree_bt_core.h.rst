
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_behavior_tree_bt_core.h:

Program Listing for File bt_core.h
==================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_behavior_tree_bt_core.h>` (``hateb_local_planner/include/hateb_local_planner/behavior_tree/bt_core.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*********************************************************************
    *
    * Software License Agreement (BSD License)
    *
    * Copyright (c) 2025 LAAS/CNRS
    * All rights reserved.
    *
    *  Redistribution and use in source and binary forms, with or without
    *  modification, are permitted provided that the following conditions
    *  are met:
    *
    *   * Redistributions of source code must retain the above copyright
    *     notice, this list of conditions and the following disclaimer.
    *   * Redistributions in binary form must reproduce the above
    *     copyright notice, this list of conditions and the following
    *     disclaimer in the documentation and/or other materials provided
    *     with the distribution.
    *   * Neither the name of the institute nor the names of its
    *     contributors may be used to endorse or promote products derived
    *     from this software without specific prior written permission.
    *
    *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    *  POSSIBILITY OF SUCH DAMAGE.
    *
    * Author: Phani Teja Singamaneni (email:ptsingaman@laas.fr)
    *********************************************************************/
   
   #ifndef BT_CORE_HH_
   #define BT_CORE_HH_
   
   #include <ros/ros.h>
   
   #include "behaviortree_cpp/bt_factory.h"
   
   #define EPS 0.01       // Small epsilon value for floating-point comparisons
   #define DIST_EPS 0.06  // Distance threshold for proximity checks
   
   #define BTPRINT 0
   
   #if BTPRINT
   #define BT_INFO(x, y) std::cout << "BT_INFO: " << x << " -> " << y << std::endl;
   
   #define BT_WARN(x, y)                                           \
     std::cout << "\033[33m";                                      \
     std::cout << "BT_WARNING: " << x << " -> " << y << std::endl; \
     std::cout << "\033[0m";
   
   #define BT_ERROR(x, y)                                        \
     std::cout << "\033[31m";                                    \
     std::cout << "BT_ERROR: " << x << " -> " << y << std::endl; \
     std::cout << "\033[0m";
   
   #else
   #define BT_INFO(x, y)
   #define BT_WARN(x, y)
   #define BT_ERROR(x, y)
   #endif
   
   inline double normalize_angle(double angle_radians) { return angle_radians - (2.0 * M_PI * std::floor((angle_radians + (M_PI)) / (2.0 * M_PI))); }
   
   /*This part of the code is inspired from here: https://github.com/BehaviorTree/BehaviorTree.ROS (bt_action_node.hh)*/
   namespace hateb_local_planner {
   
   class StatefulActionNodeROS : public BT::StatefulActionNode {
    protected:
     StatefulActionNodeROS(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration& conf) : BT::StatefulActionNode(name, conf), node_(nh) {}
   
    public:
     // using BaseClass = StatefulActionNodeROS<ActionT>;
     // using ActionType = ActionT;
   
     StatefulActionNodeROS() = delete;
   
     ~StatefulActionNodeROS() override = default;
   
     static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("action_name")}; }
   
     BT::NodeStatus onStart() override = 0;
   
     BT::NodeStatus onRunning() override = 0;
   
     void onHalted() override = 0;
   
    protected:
     ros::NodeHandle& node_;  // ROS node handle for communication
   };
   
   template <class DerivedT>
   static void RegisterStatefulActionNodeROS(BT::BehaviorTreeFactory& factory, const std::string& registration_ID, ros::NodeHandle& node_handle) {
     BT::NodeBuilder builder = [&node_handle](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<DerivedT>(node_handle, name, config); };
   
     BT::TreeNodeManifest manifest;
     manifest.type = BT::getType<DerivedT>();
     manifest.ports = DerivedT::providedPorts();
     manifest.registration_ID = registration_ID;
     const auto& basic_ports = StatefulActionNodeROS::providedPorts();
     manifest.ports.insert(basic_ports.begin(), basic_ports.end());
     factory.registerBuilder(manifest, builder);
   };
   
   enum PLAN : std::uint8_t {
     SINGLE_BAND,  
     DUAL_BAND,    
     VELOBS,       
     BACKOFF,      
     PASSTHROUGH   
   };
   
   enum PREDICTION : std::uint8_t {
     CONST_VEL,  
     BEHIND,     
     PREDICT,    
     EXTERNAL    
   };
   
   // 'Using' leads to linkage errors
   typedef struct {
     PLAN plan;           
     PREDICTION predict;  
   } ModeInfo;
   
   }  // namespace hateb_local_planner
   
   #endif  // BT_CORE_HH_
