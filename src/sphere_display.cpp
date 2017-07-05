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

SphereDisplay::SphereDisplay() :
	Display(),
	texture_front_(NULL),
	texture_rear_(NULL),
	sphere_node_(NULL),
	new_front_image_arrived_(false),
	new_rear_image_arrived_(false)
{
  image_topic_front_property_ = new RosTopicProperty("Front camera image", "",
      QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
      "Image topic of the front camera to subscribe to.",
      this, SLOT(onImageTopicChanged()));
  image_topic_rear_property_ = new RosTopicProperty("Rear camera image", "",
      QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
      "Image topic of the rear camera to subscribe to.",
      this, SLOT(onImageTopicChanged()));
  tf_frame_property_ = new TfFrameProperty("Reference frame", "<Fixed Frame>",
      "Position the sphere relative to this frame.",
      this, 0, true);

  fov_front_property_ = new FloatProperty("FOV front", 190.0,
      "Front camera field of view", this);

  fov_rear_property_= new FloatProperty("FOV rear", 190.0,
      "Rear camera field of view", this);
}

SphereDisplay::~SphereDisplay()
{
  unsubscribe();
  delete texture_front_;
  delete texture_rear_;
  delete image_topic_front_property_;
  delete image_topic_rear_property_;
  delete tf_frame_property_;
  delete fov_front_property_;
  delete fov_rear_property_;
}

void SphereDisplay::onInitialize()
{
	tf_frame_property_->setFrameManager(context_->getFrameManager());
	createSphere();
	Display::onInitialize();
}


void SphereDisplay::createSphere()
{
	// Return if node already exists.
	if(scene_manager_->hasSceneNode("sphere_display_node"))
	{
		return; 
	}

	Ogre::String node_name("sphere_display_node");
	Ogre::String entity_name("sphere_display_mesh");
	Ogre::String material_name("sphere_display_material");

	sphere_material_ = Ogre::MaterialManager::getSingleton().create(material_name,
			Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	sphere_material_->setReceiveShadows(false);
	sphere_material_->getTechnique(0)->setLightingEnabled(false);
	Ogre::Pass* pass = sphere_material_->getTechnique(0)->getPass(0);
	pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
	//		material->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);
	//		material->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_WIREFRAME);

	// Create sphere node and and add mesh entity to the scene
	sphere_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode(
			node_name, Ogre::Vector3( 0, 0, 0  ));
	sphere_node_->setScale(10,10,10);
	Ogre::Entity* sphere_entity = scene_manager_->createEntity(entity_name, Ogre::SceneManager::PT_SPHERE);
	sphere_entity->setMaterialName(material_name);
	sphere_node_->attachObject(sphere_entity);
}

void SphereDisplay::updateTexture(ROSImageTexture*& texture)
{
	if(!texture)
	{
		return;
	}

	Ogre::String texture_name = texture->getTexture()->getName();
	try
	{
		texture->update();
	}
	catch (UnsupportedImageEncoding& e)
	{
		//setStatus(StatusProperty::Error, "Front camera image", e.what());
		ROS_ERROR("SphereDisplay::updateTexture[%s]: %s", texture_name.c_str(), e.what());
		return;
	}

	// RViz texture successfully updated

	if(sphere_material_.isNull())
	{
		ROS_ERROR("SphereDisplay::updateTexture[%s]: sphere_material_ is NULL!", texture_name.c_str());
		return;
	}

	// Create Ogre textureUnitState if one does not exist already.
	Ogre::Pass* pass = sphere_material_->getTechnique(0)->getPass(0);
	ROS_ERROR("%s %i",texture_name.c_str(),pass->getTextureUnitState(texture_name));
	if(!pass->getTextureUnitState(texture_name)){
		Ogre::TextureUnitState* unit_state = pass->createTextureUnitState(texture_name);
		unit_state->setTextureAddressingMode(Ogre::TextureUnitState::TAM_BORDER);
		unit_state->setTextureScale(0.5,1);


		if(texture_name=="ROSImageTexture1")
		{
			unit_state->setColourOperation(Ogre::LBO_ADD);
		}
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

void SphereDisplay::subscribe()
{
	if (!isEnabled())
	{
		return;
	}

	if (!image_topic_front_property_->getTopic().isEmpty())
	{
		try
		{
			image_sub_ = nh_.subscribe(image_topic_front_property_->getTopicStd(),
					1, &SphereDisplay::updateFrontCameraImage, this);
			setStatus(StatusProperty::Ok, "Front camera image", "OK");
		}
		catch (ros::Exception& e)
		{
			setStatus(StatusProperty::Error, "Front camera image", QString("Error subscribing: ") + e.what());
		}
	}

	if (!image_topic_rear_property_->getTopic().isEmpty())
	{
		try
		{
			image_sub_ = nh_.subscribe(image_topic_rear_property_->getTopicStd(),
					1, &SphereDisplay::updateRearCameraImage, this);
		} catch (ros::Exception& e)
		{
			setStatus(StatusProperty::Error, "Rear camera image", QString("Error subscribing: ") + e.what());
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

	// Update front texture
	if(cur_image_front_ && new_front_image_arrived_)
	{
		ROS_INFO("update front texture %p", texture_front_);
		imageToTexture(texture_front_, cur_image_front_);
		updateTexture(texture_front_);
		new_front_image_arrived_ = false;
	}	

	// Update rear texture
	if(cur_image_rear_ && new_rear_image_arrived_)
	{
		ROS_INFO("update front texture %p", texture_front_);
		imageToTexture(texture_rear_, cur_image_rear_);
		updateTexture(texture_rear_);
		new_rear_image_arrived_ = false;
	}	

	context_->queueRender();

	if(sphere_node_)
	{
		sphere_node_->needUpdate();
	}
}


void SphereDisplay::reset()
{
//  Display::reset();
//  clear();
}

void SphereDisplay::imageToTexture(ROSImageTexture*& texture, const sensor_msgs::Image::ConstPtr& msg)
{
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
	if (texture==0)
	{
		texture = new ROSImageTexture();
		ROS_INFO("Creating new texture: %s", texture->getTexture()->getName().c_str());
	}

//	if(texture)
//	{
//		texture->addMessage(cv_ptr->toImageMsg());
//	}
//	else
//	{
//		ROS_ERROR("Failed to create texture.");
//	}

}

}  // namespace rviz




#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::SphereDisplay, rviz::Display)
