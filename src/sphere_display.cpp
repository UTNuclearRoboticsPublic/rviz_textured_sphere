/* Copyright (c) 2017 Nuclear and Applied Robotics Group, University of Texas at Austin.
 * SphereDisplay class implementation.
 *
 * Author: Veiko Vunder 
 *
 * Based on the rviz_textured_quads package by Felipe Bacim..
 *
 */

#include <OGRE/OgreArchive.h>
#include <OGRE/OgreFrustum.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreMovableObject.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
//#include <OGRE/OgreHardwarePixelBuffer.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/camera_common.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
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
#include <rviz_textured_sphere/sphere_display.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <vector>

namespace rviz
{

bool validateFloats(const sensor_msgs::CameraInfo& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.D);
  valid = valid && validateFloats(msg.K);
  valid = valid && validateFloats(msg.R);
  valid = valid && validateFloats(msg.P);
  return valid;
}

SphereDisplay::SphereDisplay()
  : Display()
  , textures_(NULL)
  , sphere_node_(NULL)
  , new_image_arrived_(false)
{
  image_topic_property_ = new RosTopicProperty("Image Topic", "",
      QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
      "Image topic to subscribe to.",
      this, SLOT(updateDisplayImages()));
  tf_frame_property_ = new TfFrameProperty("Quad Frame", "map",
      "Align the image quad to the xy plane of this tf frame",
      this, 0, true);

  meters_per_pixel_property_ = new FloatProperty("Meters per pixel", 0.002,
      "Rviz meters per image pixel.", this);
}

SphereDisplay::~SphereDisplay()
{
  unsubscribe();
  delete textures_;

  delete image_topic_property_;
  delete tf_frame_property_;
  delete meters_per_pixel_property_;
}

void SphereDisplay::onInitialize()
{
  tf_frame_property_->setFrameManager(context_->getFrameManager());
  Display::onInitialize();
}

void SphereDisplay::loadSphere(const sensor_msgs::Image::ConstPtr& image)
{
    processImage(0, *image);

	if(!scene_manager_->hasSceneNode("sphere_display_node"))
	{			
		Ogre::String node_name("sphere_display_node");
		Ogre::String entity_name("sphere_display_mesh");
		Ogre::String material_name("sphere_display_material");
		Ogre::String texture_name_front = texture_front_->getTexture()->getName();
		Ogre::String texture_name_rear = texture_rear_->getTexture()->getName();

		Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
				material_name,
				Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
		material->setReceiveShadows(false);
		material->getTechnique(0)->setLightingEnabled(false);
		Ogre::Pass* pass = material->getTecnique(0)->getPass(0);
		pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
//		material->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);
//		material->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_WIREFRAME);

		// Set up texture state for front camera image
		Ogre::TextureUnitState* unit_state_front = pass->createTextureUnitState(texture_name_front);
		unit_state_front->setTextureScale(0.5,1);
		unit_state_front->setTextureAddressingMode(Ogre::TextureUnitState::TAM_BORDER);

		// Set up texture state for rear camera image
		Ogre::TextureUnitState* unit_state_rear = pass->addTextureUnitState(texture_name_rear);
		unit_state_front->setTextureScale(0.5,1);
		unit_state_rear->setTextureAddressingMode(Ogre::TextureUnitState::TAM_BORDER);

		// Create sphere node and and add mesh entity to the scene
		sphere_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode(node_name, Ogre::Vector3( 0, 0, 0  ));
		sphere_node_->setScale(10,10,10);
		Ogre::Entity* sphere_entity = scene_manager_->createEntity(entity_name, Ogre::SceneManager::PT_SPHERE);
		sphere_entity->setMaterialName(material_name);
		sphere_node_->attachObject(sphere_entity);
	}
}


void SphereDisplay::updateImage(const sensor_msgs::Image::ConstPtr& image)
{
 cur_image_ = image;
 new_image_arrived_ = true;
}


void SphereDisplay::updateDisplayImages()
{
  unsubscribe();
  subscribe();
}

void SphereDisplay::subscribe()
{
  if (!isEnabled())
  {
    return;
  }

  if (!image_topic_property_->getTopic().isEmpty())
  {
    try
    {
      image_sub_ = nh_.subscribe(image_topic_property_->getTopicStd(),
          1, &SphereDisplay::updateImage, this);
      setStatus(StatusProperty::Ok, "Display Images Topic", "OK");
    }
    catch (ros::Exception& e)
    {
      setStatus(StatusProperty::Error, "Display Images Topic", QString("Error subscribing: ") + e.what());
    }
  }
}

void SphereDisplay::unsubscribe()
{
  image_sub_.shutdown();
}


void SphereDisplay::onEnable()
{
  subscribe();
}

void SphereDisplay::onDisable()
{
  unsubscribe();
}

void SphereDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
}

void SphereDisplay::update(float wall_dt, float ros_dt)
{
  context_->queueRender();
  if (cur_image_)
  {
	loadSphere(cur_image_);
    new_image_arrived_ = false;
  }

  if (textures_ && !image_topic_property_->getTopic().isEmpty())
  {
    try
    {
      textures_->update();
    }
    catch (UnsupportedImageEncoding& e)
    {
      setStatus(StatusProperty::Error, "Display Image", e.what());
    }
  }
  else
  {
	  ROS_INFO("no textures_ or empty image_topic_property_");
  }

}


void SphereDisplay::reset()
{
//  Display::reset();
//  clear();
}

void SphereDisplay::processImage(int index, const sensor_msgs::Image& msg)
{
   std::cout<<"camera image received with idx:"<<index<<std::endl;
  cv_bridge::CvImagePtr cv_ptr;

  // simply converting every image to RGBA
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGBA8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("SphereDisplay: cv_bridge exception: %s", e.what());
    return;
  }

  // add completely white transparent border to the image so that it won't replicate colored pixels all over the mesh
  // cv::Scalar value(255, 255, 255, 0);
  //cv::copyMakeBorder(cv_ptr->image, cv_ptr->image, 1, 1, 1, 1, cv::BORDER_CONSTANT, value);
  //cv::flip(cv_ptr->image, cv_ptr->image, -1);

  // Output modified video stream
  if (textures_ == NULL)
    textures_ = new ROSImageTexture();

  textures_->addMessage(cv_ptr->toImageMsg());
}

void SphereDisplay::load()
{

}

}  // namespace rviz




#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::SphereDisplay, rviz::Display)
