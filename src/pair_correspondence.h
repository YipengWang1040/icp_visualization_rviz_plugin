/*
 * Copyright (c) 2023, ARCS Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the ARCS Lab nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARCS_RVIZ_POINT_CLOUD2_NORMAL_DISPLAY_H
#define ARCS_RVIZ_POINT_CLOUD2_NORMAL_DISPLAY_H


#include <rviz/message_filter_display.h>
#include "icp_visualization_rviz_plugin/iCPCorrespondence.h"

namespace rviz {
class BoolProperty;
class Display;
class DisplayContext;
class EnumProperty;
class FloatProperty;
class IntProperty;
class ColorProperty;
class Line2;

/**
 * \class PointCloud2NormalDisplay
 * \brief Displays normals of a point cloud of type sensor_msgs::PointCloud2
 */
class PointCloudCorrespondenceDisplay : public MessageFilterDisplay<icp_visualization_rviz_plugin::iCPCorrespondence> {
  Q_OBJECT
 public:
  PointCloudCorrespondenceDisplay();
  ~PointCloudCorrespondenceDisplay() override;

  void reset() override;

  void update(float wall_dt, float ros_dt) override;
  void allocateLines(size_t size);

  std::vector<Ogre::Vector3> transform_pointcloud(const sensor_msgs::PointCloud2& point_cloud, const Ogre::Matrix4& transform, const std::vector<int>& indices);

private Q_SLOTS:
  void updateStyle();

protected:
  /** @brief Do initialization. Overridden from MessageFilterDisplay. */
  void onInitialize() override;

  /** @brief Process a single message.  Overridden from MessageFilterDisplay. */
  void processMessage(const icp_visualization_rviz_plugin::iCPCorrespondenceConstPtr& msg) override;

private:

  ColorProperty* start_color_property_;
  ColorProperty* end_color_property_;
  BoolProperty* gradient_color_enabled_property_;
  FloatProperty* alpha_property_;

  std::vector<rviz::Line2*> line_buffer_;
};

}  // namespace rviz

#endif