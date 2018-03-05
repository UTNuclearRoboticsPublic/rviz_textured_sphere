///////////////////////////////////////////////////////////////////////////////
//      Title     : sphere_display.h
//      Project   : rviz_textured_sphere
//      Created   : 7/13/2017
//      Author    : Veiko Vunder
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2017-2018. All rights reserved.
//
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef SPHERE_DISPLAY_H
#define SPHERE_DISPLAY_H

#include <QObject>
// kinetic compatibility http://answers.ros.org/question/233786/parse-error-at-boost_join/
#ifndef Q_MOC_RUN

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMesh.h>
#include <OGRE/OgreRenderQueueListener.h>
#include <OGRE/OgreRenderTargetListener.h>
#include <OGRE/OgreWindowEventUtilities.h>

#include <rviz/display.h>
#include <rviz/frame_manager.h>
#include <rviz/image/image_display_base.h>
#include <rviz/image/ros_image_texture.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <shape_msgs/Mesh.h>
#include <image_transport/image_transport.h>
#include <set>

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
class SphereDisplay : public rviz::Display,
                      public Ogre::RenderTargetListener,
                      public Ogre::RenderQueueListener
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
  void fillTransportOptionList(EnumProperty* enum_property);

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
  Ogre::MeshPtr createSphereMesh(const std::string& mesh_name, const double r,
                                 const unsigned int ring_cnt, const unsigned int segment_cnt);
  void destroySphere();
  void imageToTexture(ROSImageTexture*& texture, const sensor_msgs::Image::ConstPtr& msg);
  void clearStates();

  // Property objects
  RosTopicProperty* image_topic_front_property_;
  EnumProperty* front_transport_property_;
  RosTopicProperty* image_topic_rear_property_;
  EnumProperty* rear_transport_property_;
  TfFrameProperty* ref_frame_property_;
  FloatProperty* radius_property_;
  IntProperty* ring_cnt_property_;
  IntProperty* segment_cnt_property_;
  FloatProperty* fov_front_property_;
  FloatProperty* fov_rear_property_;
  FloatProperty* blend_angle_property_;

  // Image transport
  std::unique_ptr<image_transport::ImageTransport> it_front_;
  std::unique_ptr<image_transport::ImageTransport> it_rear_;
  std::shared_ptr<image_transport::SubscriberFilter> sub_front_;
  std::shared_ptr<image_transport::SubscriberFilter> sub_rear_;
  std::set<std::string> transport_plugin_types_;

  // Ogre and textures
  Ogre::SceneNode* sphere_node_;
  Ogre::MaterialPtr sphere_material_;
  ROSImageTexture* texture_front_;  // Texture for front camera image
  ROSImageTexture* texture_rear_;   // Texture for rear camera image
  RenderPanel* render_panel_;       // this is the active render panel

  bool new_front_image_arrived_;
  bool new_rear_image_arrived_;
  sensor_msgs::Image::ConstPtr cur_image_front_;
  sensor_msgs::Image::ConstPtr cur_image_rear_;
  ros::NodeHandle nh_;
};

}  // namespace rviz

#endif  // SPHERE_DISPLAY_H
