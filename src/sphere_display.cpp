///////////////////////////////////////////////////////////////////////////////
//      Title     : sphere_display.cpp
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

#include <OGRE/OgreArchive.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreMeshManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreHardwareVertexBuffer.h>
#include <OGRE/OgreHardwareBufferManager.h>
#include <OGRE/OgreSubMesh.h>
#include <OGRE/OgreAxisAlignedBox.h>
#include <OGRE/OgreVector3.h>

#include <rviz_textured_sphere/sphere_display.h>
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/render_panel.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/tf_link_updater.h>
#include <rviz/validate_floats.h>
#include <rviz/view_manager.h>
#include <rviz/visualization_manager.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/camera_common.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <angles/angles.h>
#include <sensor_msgs/image_encodings.h>

#include <pluginlib/class_loader.h>
#include <image_transport/subscriber_plugin.h>
#include <regex>
#include <string>
#include <vector>
#include <QSignalMapper>

namespace rviz
{
// bool validateFloats(const sensor_msgs::CameraInfo& msg)
//{
//  bool valid = true;
//  valid = valid && validateFloats(msg.D);
//  valid = valid && validateFloats(msg.K);
//  valid = valid && validateFloats(msg.R);
//  valid = valid && validateFloats(msg.P);
//  return valid;
//}

SphereDisplay::SphereDisplay()
  : Display()
  , sub_front_()
  , sub_rear_()
  , sphere_node_(NULL)
  , texture_front_(NULL)
  , texture_rear_(NULL)
  , new_front_image_arrived_(false)
  , new_rear_image_arrived_(false)
{
  image_topic_front_property_ = new RosTopicProperty(
      "Front camera image", "",
      QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
      "Image topic of the front camera to subscribe to.", this, SLOT(onImageTopicChanged()));

  front_transport_property_ =
      new EnumProperty("Transport Hint", "raw", "Preferred method of sending images.", this,
                       SLOT(onImageTopicChanged()));
  connect(front_transport_property_, SIGNAL(requestOptions(EnumProperty*)), this,
          SLOT(fillTransportOptionList(EnumProperty*)));

  image_topic_rear_property_ = new RosTopicProperty(
      "Rear camera image", "",
      QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
      "Image topic of the rear camera to subscribe to.", this, SLOT(onImageTopicChanged()));

  rear_transport_property_ =
      new EnumProperty("Transport Hint", "raw", "Preferred method of sending images.", this,
                       SLOT(onImageTopicChanged()));
  connect(rear_transport_property_, SIGNAL(requestOptions(EnumProperty*)), this,
          SLOT(fillTransportOptionList(EnumProperty*)));

  ref_frame_property_ =
      new TfFrameProperty("Reference frame", "<Fixed Frame>",
                          "Position the sphere relative to this frame.", this, 0, true);

  fov_front_property_ = new FloatProperty("FOV front", 235.0, "Front camera field of view", this,
                                          SLOT(onMeshParamChanged()));

  fov_rear_property_ = new FloatProperty("FOV rear", 235.0, "Rear camera field of view", this,
                                         SLOT(onMeshParamChanged()));

  blend_angle_property_ =
      new FloatProperty("Blend angle", 20, "Specifies the size of a region, where two images "
                                           "overlap and are blended together",
                        this, SLOT(onMeshParamChanged()));

  // Create and load a separate resourcegroup
  std::string path_str = ros::package::getPath(ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(path_str + "/ogre_media",
                                                                 "FileSystem", ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(ROS_PACKAGE_NAME);

  // try to create texture_unit_state
  texture_front_ = new ROSImageTexture();
  texture_rear_ = new ROSImageTexture();
  if (!texture_front_ || !texture_rear_)
  {
    ROS_ERROR("Failed to create ROSImageTextures.");
  }
  else
  {
    ROS_INFO("Created new texture: %s", texture_front_->getTexture()->getName().c_str());
    ROS_INFO("Created new texture: %s", texture_rear_->getTexture()->getName().c_str());
  }
}

SphereDisplay::~SphereDisplay()
{
  unsubscribe();
  Ogre::String node_name(ROS_PACKAGE_NAME "_node");
  Ogre::String mesh_name(ROS_PACKAGE_NAME "_mesh");
  scene_manager_->getRootSceneNode()->removeAndDestroyChild(node_name);
  Ogre::MeshManager::getSingleton().remove(mesh_name);
  delete texture_front_;
  delete texture_rear_;
  delete image_topic_front_property_;
  delete image_topic_rear_property_;
  delete ref_frame_property_;
  delete fov_front_property_;
  delete fov_rear_property_;
  delete blend_angle_property_;
  Ogre::ResourceGroupManager::getSingleton().destroyResourceGroup(ROS_PACKAGE_NAME);
}

void SphereDisplay::onInitialize()
{
  it_front_.reset(new image_transport::ImageTransport(nh_));
  it_rear_.reset(new image_transport::ImageTransport(nh_));
  ref_frame_property_->setFrameManager(context_->getFrameManager());
  createSphere();
  scanForTransportSubscriberPlugins();
}

void SphereDisplay::createSphere()
{
  Ogre::String node_name(ROS_PACKAGE_NAME "_node");
  Ogre::String material_name(ROS_PACKAGE_NAME "_material");

  // check if node already exists.
  if (scene_manager_->hasSceneNode(node_name))
  {
    return;
  }

  sphere_material_ =
      Ogre::MaterialManager::getSingleton().getByName(material_name, ROS_PACKAGE_NAME);
  if (sphere_material_.isNull())
  {
    ROS_ERROR("Unable to get OGRE material '%s'", material_name.c_str());
    return;
  }
  sphere_material_->setReceiveShadows(false);
  sphere_material_->getTechnique(0)->setLightingEnabled(false);

  // Create sphere node and and add mesh entity to the scene
  sphere_node_ =
      scene_manager_->getRootSceneNode()->createChildSceneNode(node_name, Ogre::Vector3(0, 0, 0));

  Ogre::MeshPtr sphere_mesh = createSphereMesh(ROS_PACKAGE_NAME "_mesh", 10, 64, 64);
  Ogre::Entity* sphere_entity = scene_manager_->createEntity(sphere_mesh);
  sphere_entity->setMaterialName(material_name);
  sphere_node_->attachObject(sphere_entity);
}

Ogre::MeshPtr SphereDisplay::createSphereMesh(const std::string& mesh_name, const double r,
                                              const unsigned int ring_cnt = 32,
                                              const unsigned int segment_cnt = 32)
{
  Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(mesh_name, ROS_PACKAGE_NAME);
  Ogre::SubMesh* sub_mesh = mesh->createSubMesh();
  mesh->sharedVertexData = new Ogre::VertexData();
  Ogre::VertexData* vertex_data = mesh->sharedVertexData;

  // Define vertex format
  Ogre::VertexDeclaration* vertex_decl = vertex_data->vertexDeclaration;
  size_t cur_offset = 0;
  vertex_decl->addElement(0, cur_offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  cur_offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
  vertex_decl->addElement(0, cur_offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
  cur_offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
  vertex_decl->addElement(0, cur_offset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES, 0);
  cur_offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);
  vertex_decl->addElement(0, cur_offset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES, 1);
  cur_offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);
  vertex_decl->addElement(0, cur_offset, Ogre::VET_FLOAT4, Ogre::VES_DIFFUSE);
  cur_offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT4);

  // Allocate vertex buffer
  vertex_data->vertexCount = (ring_cnt + 1) * (segment_cnt + 1);
  Ogre::HardwareVertexBufferSharedPtr v_buf =
      Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
          vertex_decl->getVertexSize(0), vertex_data->vertexCount,
          Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
  Ogre::VertexBufferBinding* binding = vertex_data->vertexBufferBinding;
  binding->setBinding(0, v_buf);
  float* vertex = static_cast<float*>(v_buf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  // Allocate index buffer
  sub_mesh->indexData->indexCount = 6 * ring_cnt * (segment_cnt + 1);
  sub_mesh->indexData->indexBuffer = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
      Ogre::HardwareIndexBuffer::IT_16BIT, sub_mesh->indexData->indexCount,
      Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
  Ogre::HardwareIndexBufferSharedPtr i_buf = sub_mesh->indexData->indexBuffer;
  unsigned short* indices =
      static_cast<unsigned short*>(i_buf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  float delta_ring_angle = M_PI / ring_cnt;
  float delta_segment_angle = 2 * M_PI / segment_cnt;
  float front_lens_fov = angles::from_degrees(fov_front_property_->getFloat());
  float rear_lens_fov = angles::from_degrees(fov_rear_property_->getFloat());
  float blend_angle = angles::from_degrees(blend_angle_property_->getFloat());
  unsigned short vertex_index = 0;

  // For over the rings of the sphere
  for (uint32_t ring = 0; ring <= ring_cnt; ring++)
  {
    float r0 = r * sinf(ring * delta_ring_angle);
    float y0 = r * cosf(ring * delta_ring_angle);

    // For over the segments of the sphere
    for (uint32_t seg = 0; seg <= segment_cnt; seg++)
    {
      float x0 = r0 * sinf(seg * delta_segment_angle);
      float z0 = r0 * cosf(seg * delta_segment_angle);

      // Calculate uv coordinates
      float v_angle = ring * delta_ring_angle;
      float u_angle = seg * delta_segment_angle;
      float uv_r0 = (float)ring / (float)ring_cnt * 2;

      // Scale and scroll textures so that their centers will align
      // with the top and bottom centers of the sphere
      float scale_front = M_PI / front_lens_fov;
      float v_front = uv_r0 * cos(u_angle) * 0.5 * scale_front + 0.5;
      float u_front = -uv_r0 * sin(u_angle) * 0.5 * scale_front + 0.5;

      float scale_rear = M_PI / rear_lens_fov;
      float v_rear = (2 - uv_r0) * cos(u_angle) * 0.5 * scale_rear + 0.5;
      float u_rear = (2 - uv_r0) * sin(u_angle) * 0.5 * scale_rear + 0.5;

      // Use diffuse color to alpha blend front and back images at the predefined blending region
      float blend_alpha = (v_angle - M_PI_2 + blend_angle / 2) / blend_angle;
      blend_alpha = fmax(fmin(blend_alpha, 1.0), 0.0);  // clamp value to [0...1]

      // Vertex position
      *vertex++ = x0;
      *vertex++ = y0;
      *vertex++ = z0;

      // Vertex normal (pointing inwards)
      Ogre::Vector3 normal = -Ogre::Vector3(x0, y0, z0).normalisedCopy();
      *vertex++ = normal.x;
      *vertex++ = normal.y;
      *vertex++ = normal.z;

      // TexCoord 0 (front image)
      *vertex++ = u_front;
      *vertex++ = v_front;

      // TexCoord 1 (rear image)
      *vertex++ = u_rear;
      *vertex++ = v_rear;

      // Set diffuse color for alpha blending
      *vertex++ = blend_alpha;  // r
      *vertex++ = blend_alpha;  // g
      *vertex++ = blend_alpha;  // b
      *vertex++ = blend_alpha;  // a

      // Add faces (normal inwards)
      if (ring != ring_cnt)
      {
        *indices++ = vertex_index + segment_cnt + 1;
        *indices++ = vertex_index + segment_cnt;
        *indices++ = vertex_index;
        *indices++ = vertex_index + 1;
        *indices++ = vertex_index + segment_cnt + 1;
        *indices++ = vertex_index;
        vertex_index++;
      }
    }
  }

  // Unlock buffers
  v_buf->unlock();
  i_buf->unlock();

  // define sphere bounds
  sub_mesh->useSharedVertices = true;
  mesh->_setBounds(Ogre::AxisAlignedBox(Ogre::Vector3(-r, -r, -r), Ogre::Vector3(r, r, r)), false);
  mesh->_setBoundingSphereRadius(r);
  mesh->load();

  return mesh;
}

void SphereDisplay::scanForTransportSubscriberPlugins()
{
  pluginlib::ClassLoader<image_transport::SubscriberPlugin> sub_loader(
      "image_transport", "image_transport::SubscriberPlugin");

  // Subscriber class names have the following format "image_transport/compressed_sub"
  // The following regex will cut off _sub and everything before /, leaving only the
  // transport name
  for (auto lookup_name : sub_loader.getDeclaredClasses())
  {
    std::regex rgx("/([a-zA-Z0-9]+)_sub");
    std::smatch matches;

    if (std::regex_search(lookup_name, matches, rgx) && matches.size() > 1)
    {
      transport_plugin_types_.insert(matches[1].str());
    }
  }
}

void SphereDisplay::fillTransportOptionList(EnumProperty* enum_property)
{
  // get the topic based on the sender
  const std::string& topic = (enum_property == front_transport_property_) ?
                                 image_topic_front_property_->getStdString() :
                                 image_topic_rear_property_->getStdString();

  enum_property->clearOptions();
  std::vector<std::string> choices;
  choices.push_back("raw");

  // Loop over all current ROS topic names
  ros::master::V_TopicInfo topics;
  ros::master::getTopics(topics);
  ros::master::V_TopicInfo::iterator it = topics.begin();
  ros::master::V_TopicInfo::iterator end = topics.end();
  for (; it != end; ++it)
  {
    // If the beginning of this topic name is the same as topic_,
    // and the whole string is not the same,
    // and the next character is /
    // and there are no further slashes from there to the end,
    // then consider this a possible transport topic.
    const ros::master::TopicInfo& ti = *it;
    const std::string& topic_name = ti.name;

    if (topic_name.find(topic) == 0 && topic_name != topic && topic_name[topic.size()] == '/' &&
        topic_name.find('/', topic.size() + 1) == std::string::npos)
    {
      std::string transport_type = topic_name.substr(topic.size() + 1);

      // If the transport type string found above is in the set of
      // supported transport type plugins, add it to the list.
      if (transport_plugin_types_.find(transport_type) != transport_plugin_types_.end())
      {
        choices.push_back(transport_type);
      }
    }
  }

  for (size_t i = 0; i < choices.size(); i++)
  {
    enum_property->addOptionStd(choices[i]);
  }
}

void SphereDisplay::updateFrontCameraImage(const sensor_msgs::Image::ConstPtr& image)
{
  cur_image_front_ = image;
  new_front_image_arrived_ = true;
}

void SphereDisplay::updateRearCameraImage(const sensor_msgs::Image::ConstPtr& image)
{
  cur_image_rear_ = image;
  new_rear_image_arrived_ = true;
}

void SphereDisplay::onImageTopicChanged()
{
  unsubscribe();
  subscribe();
}

void SphereDisplay::onMeshParamChanged()
{
  // Reload sphere mesh when some of its parameters have changed
  Ogre::String node_name(ROS_PACKAGE_NAME "_node");
  Ogre::String mesh_name(ROS_PACKAGE_NAME "_mesh");
  scene_manager_->getRootSceneNode()->removeAndDestroyChild(node_name);
  Ogre::MeshManager::getSingleton().remove(mesh_name);
  createSphere();
}

void SphereDisplay::subscribe()
{
  // Subscribe to image topics
  if (!isEnabled())
  {
    return;
  }

  const uint32_t queue_size = 1;
  sub_front_.reset(new image_transport::SubscriberFilter());
  sub_rear_.reset(new image_transport::SubscriberFilter());

  if (!image_topic_front_property_->getTopicStd().empty() &&
      !front_transport_property_->getStdString().empty())
  {
    try
    {
      sub_front_->subscribe(
          *it_front_, image_topic_front_property_->getTopicStd(), queue_size,
          image_transport::TransportHints(front_transport_property_->getStdString()));
      sub_front_->registerCallback(boost::bind(&SphereDisplay::updateFrontCameraImage, this, _1));
      setStatus(StatusProperty::Ok, "Front camera image", "OK");
    }
    catch (ros::Exception& e)
    {
      setStatus(StatusProperty::Error, "Front camera image",
                QString("Error subscribing: ") + e.what());
    }
    catch (image_transport::Exception& e)
    {
      setStatus(StatusProperty::Error, "Front camera image",
                QString("Error subscribing: ") + e.what());
    }
  }

  if (!image_topic_rear_property_->getTopicStd().empty() &&
      !rear_transport_property_->getStdString().empty())
  {
    try
    {
      sub_rear_->subscribe(
          *it_rear_, image_topic_rear_property_->getTopicStd(), queue_size,
          image_transport::TransportHints(rear_transport_property_->getStdString()));
      sub_rear_->registerCallback(boost::bind(&SphereDisplay::updateRearCameraImage, this, _1));
    }
    catch (ros::Exception& e)
    {
      setStatus(StatusProperty::Error, "Rear camera image",
                QString("Error subscribing: ") + e.what());
    }
    catch (image_transport::Exception& e)
    {
      setStatus(StatusProperty::Error, "Front camera image",
                QString("Error subscribing: ") + e.what());
    }
  }
}

void SphereDisplay::unsubscribe()
{
  sub_front_.reset(new image_transport::SubscriberFilter());
  sub_rear_.reset(new image_transport::SubscriberFilter());
}

void SphereDisplay::onEnable()
{
  subscribe();
  createSphere();
}

void SphereDisplay::onDisable()
{
  unsubscribe();
  Ogre::String node_name(ROS_PACKAGE_NAME "_node");
  Ogre::String mesh_name(ROS_PACKAGE_NAME "_mesh");
  scene_manager_->getRootSceneNode()->removeAndDestroyChild(node_name);
  Ogre::MeshManager::getSingleton().remove(mesh_name);
}

void SphereDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
}

void SphereDisplay::update(float wall_dt, float ros_dt)
{
  // Update front texture
  if (new_front_image_arrived_)
  {
    imageToTexture(texture_front_, cur_image_front_);
    new_front_image_arrived_ = false;

    if (texture_front_)
    {
      try
      {
        texture_front_->update();
      }
      catch (UnsupportedImageEncoding& e)
      {
        setStatus(StatusProperty::Error, "Front camera image", e.what());
        ROS_ERROR("SphereDisplay::update: UnsupportedImageEncoding: %s", e.what());
      }
    }
  }

  // Update rear texture
  if (new_rear_image_arrived_)
  {
    imageToTexture(texture_rear_, cur_image_rear_);
    new_rear_image_arrived_ = false;

    if (texture_rear_)
    {
      try
      {
        texture_rear_->update();
      }
      catch (UnsupportedImageEncoding& e)
      {
        setStatus(StatusProperty::Error, "Rear camera image", e.what());
        ROS_ERROR("SphereDisplay::update: UnsupportedImageEncoding: %s", e.what());
      }
    }
  }

  // update sphere node position and orientation
  if (sphere_node_)
  {
    FrameManager* frame_mgr = ref_frame_property_->getFrameManager();
    Ogre::Vector3 pos;
    Ogre::Quaternion ori;

    if (frame_mgr->getTransform(ref_frame_property_->getFrameStd(), ros::Time(), pos, ori))
    {
      sphere_node_->setPosition(pos);

      // Rotate the sphere so that its top side points towards X direction in rviz space
      Ogre::Quaternion r1, r2;
      r1.FromAngleAxis(Ogre::Radian(-M_PI_2), Ogre::Vector3::UNIT_Z);
      r2.FromAngleAxis(Ogre::Radian(M_PI), Ogre::Vector3::UNIT_Y);
      sphere_node_->setOrientation(ori * r2 * r1);
    }
    else
    {
      ROS_WARN_THROTTLE(5, "Could not get transform from reference frame: %s",
                        ref_frame_property_->getFrameStd().c_str());
    }
  }

  // request sphere update
  context_->queueRender();
  if (sphere_node_)
  {
    sphere_node_->needUpdate();
  }
}

void SphereDisplay::imageToTexture(ROSImageTexture*& texture,
                                   const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;

  // convert every image to RGBA
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGBA8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("imageToTexture: cv_bridge exception: %s", e.what());
    return;
  }

  if (!texture)
  {
    ROS_ERROR("imageToTexture: Texture is NULL:");
    return;
  }

  // add arrived image to ROSImageTexture
  texture->addMessage(cv_ptr->toImageMsg());

  // check if material and render pass are loaded and
  // assign texture to ogre's texture unit state
  if (sphere_material_.isNull())
  {
    ROS_ERROR("imageToTexture(): sphere_material_ is NULL.");
    return;
  }

  Ogre::Pass* pass = sphere_material_->getTechnique(0)->getPass(0);
  if (!pass)
  {
    ROS_ERROR("imageToTexture(): pass is NULL.");
    return;
  }

  if (pass->getNumTextureUnitStates() < 2)
  {
    ROS_ERROR("imageToTexture(): Number of texture unit states is less than 2.");
    return;
  }

  bool is_front_texture = (texture == texture_front_);

  // get unit state (0-front cam, 1-rear cam)
  int unit_state_idx = is_front_texture ? 0 : 1;
  Ogre::TextureUnitState* unit_state = pass->getTextureUnitState(unit_state_idx);
  if (!unit_state)
  {
    ROS_ERROR("Failed to getTextureUnitState(%d).", unit_state_idx);
    return;
  }

  unit_state->setTexture(texture->getTexture());
}

void SphereDisplay::reset()
{
}

}  // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::SphereDisplay, rviz::Display)
