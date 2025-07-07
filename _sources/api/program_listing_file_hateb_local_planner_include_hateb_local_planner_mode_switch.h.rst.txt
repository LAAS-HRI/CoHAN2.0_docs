
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_mode_switch.h:

Program Listing for File mode_switch.h
======================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_mode_switch.h>` (``hateb_local_planner/include/hateb_local_planner/mode_switch.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef MODE_SWITCH_HH_
   #define MODE_SWITCH_HH_
   
   #include <agent_path_prediction/agents_class.h>
   #include <ros/ros.h>
   
   #include "behaviortree_cpp/bt_factory.h"
   
   // Messages
   #include <actionlib_msgs/GoalStatusArray.h>
   #include <agent_path_prediction/AgentsInfo.h>
   #include <cohan_msgs/PassageType.h>
   #include <geometry_msgs/Pose.h>
   #include <hateb_local_planner/PlanningMode.h>
   #include <move_base_msgs/MoveBaseActionGoal.h>
   #include <move_base_msgs/MoveBaseActionResult.h>
   
   // BT Nodes
   #include <hateb_local_planner/behavior_tree/action/set_mode.h>
   #include <hateb_local_planner/behavior_tree/bt_core.h>
   #include <hateb_local_planner/behavior_tree/condition/backoff_exit_condition.h>
   #include <hateb_local_planner/behavior_tree/condition/dual_band_exit_condition.h>
   #include <hateb_local_planner/behavior_tree/condition/is_goal_reached.h>
   #include <hateb_local_planner/behavior_tree/condition/is_goal_updated.h>
   #include <hateb_local_planner/behavior_tree/condition/passthrough_condition.h>
   #include <hateb_local_planner/behavior_tree/condition/single_band_exit_condition.h>
   #include <hateb_local_planner/behavior_tree/condition/vel_obs_exit_condition.h>
   
   // Backoff recovery mechanism
   #include <hateb_local_planner/backoff.h>
   
   // Stdlib
   #include <mutex>
   
   namespace hateb_local_planner {
   
   class ModeSwitch {
    public:
     ModeSwitch();
   
     ~ModeSwitch() = default;
   
     void initialize(ros::NodeHandle& nh, std::string& xml_path, std::shared_ptr<agents::Agents>& agents_ptr, std::shared_ptr<Backoff>& backoff_ptr);
   
     BT::NodeStatus tickBT();
   
     void resetBT();
   
     BT::Tree* BTree() { return &bhv_tree_; }
   
     hateb_local_planner::PlanningMode tickAndGetMode();
   
    private:
     void registerNodes();
   
     void agentsInfoCB(const agent_path_prediction::AgentsInfo& info_msg);
   
     void goalMoveBaseCB(const move_base_msgs::MoveBaseActionGoal& goal_msg);
   
     void resultMoveBaseCB(const move_base_msgs::MoveBaseActionResult& result_msg);
   
     void passageCB(const cohan_msgs::PassageType& passage_msg);
   
     void updateMode(int duration = 0);
   
     void printTreeStatus(const BT::TreeNode* node, int level = 0) {
       std::string indent(level * 2, ' ');
       std::cout << indent << node->name() << ": " << toStr(node->status()) << std::endl;
   
       if (auto control = dynamic_cast<const BT::ControlNode*>(node)) {
         for (unsigned i = 0; i < control->childrenCount(); ++i) {
           printTreeStatus(control->child(i), level + 1);
         }
       }
     }
   
     // Status flags
     bool goal_reached_;  
     bool initialized_;   
     bool goal_update_;   
   
     // ROS communication members
     ros::NodeHandle nh_;                  
     ros::Subscriber agents_info_sub_;     
     ros::Subscriber goal_sub_;            
     ros::Subscriber result_sub_;          
     ros::Subscriber passage_detect_sub_;  
     ros::Publisher planning_mode_pub_;    
     ros::ServiceServer backoff_srv_;      
   
     // State information
     geometry_msgs::PoseStamped goal_;                
     agent_path_prediction::AgentsInfo agents_info_;  
     actionlib_msgs::GoalStatusArray result_msg_;     
   
     // Behavior Tree components
     BT::BehaviorTreeFactory bhv_factory_;  
     BT::Tree bhv_tree_;                    
   
     std::mutex pub_mutex_;  
   
     std::string name_;                             
     hateb_local_planner::PlanningMode plan_mode_;  
     ModeInfo mode_info_;                           
   
     std::shared_ptr<Backoff> backoff_ptr_;  
   
     // Params for namespace and subscription topics
     std::string ns_;                     
     std::string agents_info_sub_topic_;  
     std::string goal_sub_topic_;         
     std::string result_sub_topic_;       
     std::string passage_sub_topic_;      
   };
   
   }  // namespace hateb_local_planner
   #endif  // MODE_SWITCH_HH_
