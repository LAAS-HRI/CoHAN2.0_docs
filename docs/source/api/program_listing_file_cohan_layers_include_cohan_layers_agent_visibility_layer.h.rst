
.. _program_listing_file_cohan_layers_include_cohan_layers_agent_visibility_layer.h:

Program Listing for File agent_visibility_layer.h
=================================================

|exhale_lsh| :ref:`Return to documentation for file <file_cohan_layers_include_cohan_layers_agent_visibility_layer.h>` (``cohan_layers/include/cohan_layers/agent_visibility_layer.h``)

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
   
   #ifndef AGENT_VISIBILITY_LAYER_H
   #define AGENT_VISIBILITY_LAYER_H
   #include <cohan_layers/AgentVisibilityLayerConfig.h>
   #include <cohan_layers/agent_layer.h>
   #include <dynamic_reconfigure/server.h>
   #include <ros/ros.h>
   
   namespace cohan_layers {
   class AgentVisibilityLayer : public AgentLayer {
    public:
     AgentVisibilityLayer() { layered_costmap_ = nullptr; }
   
     void onInitialize() override;
   
     void updateBoundsFromAgents(double* min_x, double* min_y, double* max_x, double* max_y) override;
   
     void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
   
    protected:
     void configure(AgentVisibilityLayerConfig& config, uint32_t level);
   
     dynamic_reconfigure::Server<AgentVisibilityLayerConfig>* server_;
   
     dynamic_reconfigure::Server<AgentVisibilityLayerConfig>::CallbackType f_;
   };
   }  // namespace cohan_layers
   
   #endif  // AGENT_VISIBILITY_LAYER_H
