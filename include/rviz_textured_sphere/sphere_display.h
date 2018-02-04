/* Copyright (c) 2013-2015 Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

#pragma once

#include <QObject>
// kinetic compatibility http://answers.ros.org/question/233786/parse-error-at-boost_join/
#ifndef Q_MOC_RUN

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreMesh.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreRenderQueueListener.h>
#include <OGRE/OgreRenderSystem.h>
#include <OGRE/OgreRenderTargetListener.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreWindowEventUtilities.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <map>
#include <message_filters/subscriber.h>
#include <rviz/display.h>
#include <rviz/frame_manager.h>
#include <rviz/image/image_display_base.h>
#include <rviz/image/ros_image_texture.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <shape_msgs/Mesh.h>
#include <std_msgs/Float64.h>
#include <tf/message_filter.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <vector>

#endif  // Q_MOC_RUN

namespace Ogre
{
class Entity;
class SceneNode;
class ManualObject;
}

namespace rviz
{
class FloatProperty;
class RenderPanel;
class RosTopicProperty;
class TfFrameProperty;

/**
 * \class SphereDisplay
 * \brief Uses a pose from topic + offset to render a bounding object with shape, size and color
 */
class SphereDisplay: public rviz::Display,  public Ogre::RenderTargetListener, public Ogre::RenderQueueListener
{
  Q_OBJECT
public:
  SphereDisplay();
  virtual ~SphereDisplay();

  // Overrides from Display
  virtual void onInitialize();
  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

  virtual void preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt);

private Q_SLOTS:
  void updateMeshProperties();
  void onImageTopicChanged();
  void onMeshParamChanged();
  void fillTransportOptionList(RosTopicProperty* topic_property);

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  void processImage(int index, const sensor_msgs::Image& msg);

  virtual void subscribe();
  virtual void unsubscribe();

private:
  void clear();
  void scanForTransportSubscriberPlugins();
  void updateFrontCameraImage(const sensor_msgs::Image::ConstPtr& image);
  void updateRearCameraImage(const sensor_msgs::Image::ConstPtr& image);
  void createSphere();
  Ogre::MeshPtr createSphereMesh(const std::string& mesh_name, const double r, const unsigned int ring_cnt, const unsigned int segment_cnt);
  void imageToTexture(ROSImageTexture*& texture, const sensor_msgs::Image::ConstPtr& msg);
  void clearStates();

  std::set<std::string> transport_plugin_types_;
  RosTopicProperty* image_topic_front_property_;
  EnumProperty* front_transport_property_;
  RosTopicProperty* image_topic_rear_property_;
  EnumProperty* rear_transport_property_;
  TfFrameProperty* ref_frame_property_;
  FloatProperty* fov_front_property_;
  FloatProperty* fov_rear_property_;
  FloatProperty* blend_angle_property_;

  ros::Subscriber image_sub_front_;
  ros::Subscriber image_sub_rear_;

  ros::NodeHandle nh_;

  bool new_front_image_arrived_;
  bool new_rear_image_arrived_;
  sensor_msgs::Image::ConstPtr cur_image_front_;
  sensor_msgs::Image::ConstPtr cur_image_rear_;

  Ogre::SceneNode* sphere_node_;
  Ogre::MaterialPtr sphere_material_;
  ROSImageTexture* texture_front_; // Texture for front camera image
  ROSImageTexture* texture_rear_; // Texture for rear camera image

  RenderPanel* render_panel_;  // this is the active render panel
};

}  // namespace rviz



