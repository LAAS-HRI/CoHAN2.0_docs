
.. _program_listing_file_rviz_path_array_src_patharray_display.h:

Program Listing for File patharray_display.h
============================================

|exhale_lsh| :ref:`Return to documentation for file <file_rviz_path_array_src_patharray_display.h>` (``rviz_path_array/src/patharray_display.h``)

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
    *  Author: Phani Teja Singamaneni (email:ptsingaman@laas.fr)
    *********************************************************************/
   
   #ifndef AGENT_PATHARRAY_DISPLAY_H
   #define AGENT_PATHARRAY_DISPLAY_H
   
   // #include <nav_msgs/Path.h>
   #include <cohan_msgs/AgentPath.h>
   #include <cohan_msgs/AgentPathArray.h>
   #include <rviz/ogre_helpers/arrow.h>
   #include <rviz/ogre_helpers/axes.h>
   
   #include "rviz/message_filter_display.h"
   
   namespace Ogre {
   class ManualObject;
   }
   
   namespace rviz {
   class ColorProperty;
   class FloatProperty;
   class IntProperty;
   class EnumProperty;
   class BillboardLine;
   class VectorProperty;
   }  // namespace rviz
   
   namespace rviz_path_array {
   
   class AgentPathArrayDisplay;
   class AgentPathArrayDisplay : public rviz::MessageFilterDisplay<cohan_msgs::AgentPathArray> {
     Q_OBJECT
    public:
     AgentPathArrayDisplay();
     ~AgentPathArrayDisplay() override;
   
     void reset() override;
   
    protected:
     void onInitialize() override;
   
     void processMessage(const cohan_msgs::AgentPathArray::ConstPtr& msg) override;
   
    private Q_SLOTS:
     void updateBufferLength();
     void updateStyle();
     void updateLineWidth();
     void updateOffset();
     void updatePoseStyle();
     void updatePoseAxisGeometry();
     void updatePoseArrowColor();
     void updatePoseArrowGeometry();
   
    private:
     void destroyObjects();
     void allocateArrowVector(std::vector<rviz::Arrow*>& arrow_vect, int num);
     void allocateAxesVector(std::vector<rviz::Axes*>& axes_vect, int num);
     void destroyPoseAxesChain();
     void destroyPoseArrowChain();
   
     std::vector<Ogre::ManualObject*> manual_objects_;
     std::vector<rviz::BillboardLine*> billboard_lines_;
     std::vector<std::vector<rviz::Axes*> > axes_chain_;
     std::vector<std::vector<rviz::Arrow*> > arrow_chain_;
   
     rviz::EnumProperty* style_property_;
     rviz::ColorProperty* color_property_;
     rviz::FloatProperty* alpha_property_;
     rviz::FloatProperty* line_width_property_;
     rviz::IntProperty* buffer_length_property_;
     rviz::VectorProperty* offset_property_;
   
     enum LineStyle { LINES, BILLBOARDS };
   
     // pose marker property
     rviz::EnumProperty* pose_style_property_;
     rviz::FloatProperty* pose_axes_length_property_;
     rviz::FloatProperty* pose_axes_radius_property_;
     rviz::ColorProperty* pose_arrow_color_property_;
     rviz::FloatProperty* pose_arrow_shaft_length_property_;
     rviz::FloatProperty* pose_arrow_head_length_property_;
     rviz::FloatProperty* pose_arrow_shaft_diameter_property_;
     rviz::FloatProperty* pose_arrow_head_diameter_property_;
   
     enum PoseStyle {
       NONE,
       AXES,
       ARROWS,
     };
   };
   
   }  // namespace rviz_path_array
   
   #endif /* AGENT_PATHARRAY_DISPLAY_H */
