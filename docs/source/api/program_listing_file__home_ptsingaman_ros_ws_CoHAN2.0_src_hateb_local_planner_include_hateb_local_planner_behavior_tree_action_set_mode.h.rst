
.. _program_listing_file__home_ptsingaman_ros_ws_CoHAN2.0_src_hateb_local_planner_include_hateb_local_planner_behavior_tree_action_set_mode.h:

Program Listing for File set_mode.h
===================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ptsingaman_ros_ws_CoHAN2.0_src_hateb_local_planner_include_hateb_local_planner_behavior_tree_action_set_mode.h>` (``/home/ptsingaman/ros_ws/CoHAN2.0/src/hateb_local_planner/include/hateb_local_planner/behavior_tree/action/set_mode.h``)

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
   
   #include <hateb_local_planner/PlanningMode.h>
   #include <hateb_local_planner/behavior_tree/bt_core.h>
   #include <ros/ros.h>
   #include <tf2/utils.h>
   
   namespace hateb_local_planner {
   
   class SetMode : public StatefulActionNodeROS {
    public:
     SetMode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfig& config);
   
     SetMode() = delete;
   
     ~SetMode() override;
   
     static BT::PortsList providedPorts() {
       // define the input and output ports
       return {BT::InputPort<std::string>("plan_type"), BT::InputPort<std::string>("predict_type"), BT::OutputPort<ModeInfo>("mode")};
     }
   
     BT::NodeStatus onStart() override;
   
     BT::NodeStatus onRunning() override;
   
     void onHalted() override;
   
    private:
     std::string name_;                  
     ros::Publisher planning_mode_pub_;  
   
     // BT Tree main
     std::string plan_type_;     
     std::string predict_type_;  
     ModeInfo p_msg_;            
   };
   };  // namespace hateb_local_planner
