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
#include <OGRE/OgreMeshManager.h>
#include <OGRE/OgreMovableObject.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreHardwareVertexBuffer.h>
#include <OGRE/OgreHardwareBufferManager.h>
#include <OGRE/OgreMesh.h>
#include <OGRE/OgreSubMesh.h>
#include <OGRE/OgreAxisAlignedBox.h>
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
#include <rviz/image/ros_image_texture.h>
#include <rviz_textured_sphere/sphere_display.h>
#include <angles/angles.h>
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

  fov_front_property_ = new FloatProperty("FOV front", 235.0,
      "Front camera field of view", this, SLOT(onMeshParamChanged()));

  fov_rear_property_= new FloatProperty("FOV rear", 235.0,
      "Rear camera field of view", this, SLOT(onMeshParamChanged()));

  blend_angle_property_= new FloatProperty("Blend angle", 0,
      "Specifies the size of a region, where two images overlap and are blended together", 
	  this, SLOT(onMeshParamChanged()));

  debug_property_= new FloatProperty("Debug value", 0.0f,
	  "A value for debugging", this, SLOT(onDebugValueChanged()));

  // Create and load a separate resourcegroup
  std::string path_str = ros::package::getPath(ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation( path_str + "/ogre_media", "FileSystem", ROS_PACKAGE_NAME  );
  Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(ROS_PACKAGE_NAME);

  //try to create texture_unit_state
  texture_front_ = new ROSImageTexture();
  texture_rear_ = new ROSImageTexture();
  if(!texture_front_ || !texture_rear_)
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
  delete texture_front_;
  delete texture_rear_;
  delete image_topic_front_property_;
  delete image_topic_rear_property_;
  delete tf_frame_property_;
  delete fov_front_property_;
  delete fov_rear_property_;
  delete blend_angle_property_;
  delete debug_property_;
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
	Ogre::String node_name(ROS_PACKAGE_NAME "_node");
	Ogre::String material_name(ROS_PACKAGE_NAME "_material");

	if(scene_manager_->hasSceneNode(node_name))
	{
		return; 
	}


	sphere_material_ = Ogre::MaterialManager::getSingleton().getByName(material_name, ROS_PACKAGE_NAME);
	if(sphere_material_.isNull())
	{
		ROS_ERROR("createSphere(): couldn't get material '%s'", material_name.c_str());
		return;
	}
	sphere_material_->setReceiveShadows(false);
	sphere_material_->getTechnique(0)->setLightingEnabled(false);
//	Ogre::Pass* pass = sphere_material_->getTechnique(0)->getPass(0);
	
	//pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
	//pass->setSceneBlending(Ogre::SBT_REPLACE);
	//pass->setSceneBlendOperation
//	sphere_material_->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);
	//		material->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_WIREFRAME);

	// Create sphere node and and add mesh entity to the scene
	sphere_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode(
			node_name, Ogre::Vector3( 0, 0, 0  ));

	    Ogre::Quaternion r1, r2; // Rotate from RViz coordinates to OpenGL coordinates
	    r1.FromAngleAxis(Ogre::Radian(M_PI*0.5), Ogre::Vector3::UNIT_X);
	    r2.FromAngleAxis(Ogre::Radian(-M_PI*0.5), Ogre::Vector3::UNIT_Y);
	sphere_node_->rotate(r1*r2);

	sphere_node_->setDirection(Ogre::Vector3(0,1,0));
//	sphere_node_->setScale(1,1,1);
	//Ogre::Entity* sphere_entity = scene_manager_->createEntity(entity_name, Ogre::SceneManager::PT_SPHERE);
	Ogre::MeshPtr sphere_mesh = createSphereMesh(ROS_PACKAGE_NAME "_mesh", 10, 64, 64);
	Ogre::Entity* sphere_entity = scene_manager_->createEntity(sphere_mesh);
	sphere_entity->setMaterialName(material_name);
	sphere_node_->attachObject(sphere_entity);
}


Ogre::MeshPtr SphereDisplay::createSphereMesh(const std::string& mesh_name, const double r, const unsigned int ring_cnt = 32, const unsigned int segment_cnt = 32)
{
	Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(mesh_name, ROS_PACKAGE_NAME);
	Ogre::SubMesh* sub_mesh = mesh -> createSubMesh();
	mesh->sharedVertexData = new Ogre::VertexData();
	Ogre::VertexData* vertex_data = mesh->sharedVertexData;

	// Define vertex format
	// Position, Normal, and UV coord0, UV coord1
	Ogre::VertexDeclaration* vertex_decl = vertex_data -> vertexDeclaration;
	size_t cur_offset = 0;
	vertex_decl -> addElement(0, cur_offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
	cur_offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
	vertex_decl -> addElement(0, cur_offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
	cur_offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
	vertex_decl -> addElement(0, cur_offset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES, 0);
	cur_offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);
	vertex_decl -> addElement(0, cur_offset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES, 1);
	cur_offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);
	vertex_decl -> addElement(0, cur_offset, Ogre::VET_FLOAT4, Ogre::VES_DIFFUSE);
	cur_offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT4);
	//vertex_decl -> addElement(0, cur_offset, Ogre::VET_FLOAT4, Ogre::VES_SPECULAR);
	//cur_offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT4);
	

	// Allocate vertex buffer
	vertex_data -> vertexCount = (ring_cnt + 1) * (segment_cnt + 1);
	Ogre::HardwareVertexBufferSharedPtr v_buf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(vertex_decl->getVertexSize(0), vertex_data->vertexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
	Ogre::VertexBufferBinding* binding = vertex_data -> vertexBufferBinding;
	binding -> setBinding(0,v_buf);
	float* vertex = static_cast<float*>(v_buf -> lock(Ogre::HardwareBuffer::HBL_DISCARD));
	
	// Allocate index buffer
	sub_mesh -> indexData -> indexCount = 6 * ring_cnt * (segment_cnt + 1);
	sub_mesh -> indexData -> indexBuffer = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
			Ogre::HardwareIndexBuffer::IT_16BIT, 
			sub_mesh -> indexData -> indexCount,
			Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
			false);
	Ogre::HardwareIndexBufferSharedPtr i_buf = sub_mesh -> indexData ->indexBuffer;
	unsigned short* indices = static_cast<unsigned short*>(i_buf -> lock(Ogre::HardwareBuffer::HBL_DISCARD));

	float delta_ring_angle = M_PI / ring_cnt;
	float delta_segment_angle = 2 * M_PI / segment_cnt;
	unsigned short vertex_index = 0;	

	// For over the rings of the sphere
	for(int ring = 0; ring <= ring_cnt; ring++)
	{
		float r0 = r * sinf(ring * delta_ring_angle);
		float y0 = r * cosf(ring * delta_ring_angle);

		// For over the segments of the sphere
		for(int seg = 0; seg <= segment_cnt; seg++)
		{
			float x0 = r0 * sinf(seg * delta_segment_angle);
			float z0 = r0 * cosf(seg * delta_segment_angle);

			// Add vertex pos
			*vertex++ = x0;
			*vertex++ = y0;
			*vertex++ = z0;

			// Add vertex normal (pointing inwards)
			Ogre::Vector3 normal = -Ogre::Vector3(x0, y0, z0).normalisedCopy();
			*vertex++ = normal.x;
			*vertex++ = normal.y;
			*vertex++ = normal.z;

			// Add uv coordinates
			float front_lens_fov = angles::from_degrees(fov_front_property_->getFloat());
			float rear_lens_fov = angles::from_degrees(fov_rear_property_->getFloat());
			float blend_angle = angles::from_degrees(blend_angle_property_->getFloat());

			float v_angle = ring*delta_ring_angle;
			float u_angle = seg*delta_segment_angle;
			float uv_r0 = sinf(v_angle);

			uv_r0 = (float)ring / (float)ring_cnt*2;

			//scale and scroll textures so that their centers will align with the centers of half spheres
			float scale_front = M_PI / front_lens_fov;
			float v_front = uv_r0*cos(u_angle)*0.5*scale_front+0.5;
			float u_front = -uv_r0*sin(u_angle)*0.5*scale_front+0.5;

			float scale_rear = M_PI / rear_lens_fov;
			float v_rear = (2-uv_r0)*cos(u_angle)*0.5*scale_rear+0.5;
			float u_rear = (2-uv_r0)*sin(u_angle)*0.5*scale_rear+0.5+debug_property_->getFloat();

			
			// create mask for front image
			if(v_angle > M_PI_2 + blend_angle/2)
			{
				Ogre::Vector2 uv_front = Ogre::Vector2(u_front,v_front);
				uv_front.normalise();
				uv_front = uv_front*1000;
				u_front = uv_front.x;
				v_front = uv_front.y;

				if (u_front==0 && v_front==0)
				{
					u_front=10;
					v_front=10;
				}
			}

			//create mask for rear image
			if(v_angle < M_PI_2 - blend_angle/2)
			{
				//map out of interest uv coordinates to a circle out of texture boundaries
				Ogre::Vector2 uv_rear = Ogre::Vector2(u_rear,v_rear);
				uv_rear.normalise();
				uv_rear = uv_rear*1000;
				u_rear = uv_rear.x;
				v_rear = uv_rear.y;

				if (u_rear==0 && v_rear==0)
				{
					u_rear=10;
					v_rear=10;
				}
			}
			//ROS_INFO("angle:%f, Varg %f, Uarg %f Varg %f, Uarg %f", v_angle, v_front, u_front, v_rear, u_rear);

			//TexCoord 0
			*vertex++ = u_front;
			*vertex++ = v_front;

			//TexCoord 1
			*vertex++ = u_rear;
			*vertex++ = v_rear;

			//Diffuse for blending
			float blend_alpha = (v_angle-M_PI_2+blend_angle/2)/blend_angle;
			if(blend_alpha<0) blend_alpha = 0;
			if(blend_alpha>1) blend_alpha = 1;
			
//			blend_alpha = debug_property_->getFloat();
			*vertex++ = blend_alpha; // r
			*vertex++ = blend_alpha; // g
			*vertex++ = blend_alpha; // b
			*vertex++ =	blend_alpha; // a
			//
			//Diffuse for blending
		//	*vertex++ = 1.0f; // r
		//	*vertex++ = 1.0f; // g
		//	*vertex++ = 1.0f; // b
		//	*vertex++ =	alpha_rear; // a
			


			// Add faces (normal inwards)
			if(ring != ring_cnt)	
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
	v_buf -> unlock();
	i_buf -> unlock();

	sub_mesh -> useSharedVertices = true;
	mesh -> _setBounds(Ogre::AxisAlignedBox(Ogre::Vector3(-r, -r, -r), Ogre::Vector3(r, r, r)), false);
	mesh -> _setBoundingSphereRadius(r);
	mesh -> load();

	return mesh;
}


void SphereDisplay::updateFrontCameraImage(const sensor_msgs::Image::ConstPtr& image)
{
	//ROS_INFO("New FRONT image arrived");
	cur_image_front_ = image;
	new_front_image_arrived_ = true;
}

void SphereDisplay::updateRearCameraImage(const sensor_msgs::Image::ConstPtr& image)
{
	//ROS_INFO("New REAR image arrived");
	cur_image_rear_ = image;
	new_rear_image_arrived_ = true;
}


void SphereDisplay::onImageTopicChanged()
{
	unsubscribe();
	subscribe();
}

void SphereDisplay::onDebugValueChanged()
{
	onMeshParamChanged();
	if(sphere_material_.isNull())
	{
		ROS_ERROR("imageToTexture(): sphere_material_ is NULL.");
		return;
	}

	Ogre::Pass* pass = sphere_material_ -> getTechnique(0) -> getPass(0);
	if (!pass)
	{
		ROS_ERROR("imageToTexture(): pass is NULL.");
		return;
	}

	if (pass->getNumTextureUnitStates()<2)
	{
		ROS_ERROR("imageToTexture(): Number of texture unit states is less than 2.");
		return;
	}
}

void SphereDisplay::onMeshParamChanged()
{
	Ogre::String node_name(ROS_PACKAGE_NAME "_node");
	Ogre::String mesh_name(ROS_PACKAGE_NAME "_mesh");
	scene_manager_->getRootSceneNode()->removeAndDestroyChild(node_name);
	Ogre::MeshManager::getSingleton().remove(mesh_name);
	createSphere();
}


void SphereDisplay::subscribe()
{
	// Subscribe to receive front and rear camera images
	if (!isEnabled())
	{
		return;
	}

	if (!image_topic_front_property_->getTopic().isEmpty())
	{
		try
		{
			image_sub_front_ = nh_.subscribe(image_topic_front_property_->getTopicStd(),
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
			image_sub_rear_ = nh_.subscribe(image_topic_rear_property_->getTopicStd(),
					1, &SphereDisplay::updateRearCameraImage, this);
		} catch (ros::Exception& e)
		{
			setStatus(StatusProperty::Error, "Rear camera image", QString("Error subscribing: ") + e.what());
		}
	}
}

void SphereDisplay::unsubscribe()
{
	//ROS_INFO("unsubscribe()");
	image_sub_front_.shutdown();
	image_sub_rear_.shutdown();
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
	if(new_front_image_arrived_)
	{
		imageToTexture(texture_front_, cur_image_front_);
		updateTexture(texture_front_);
		new_front_image_arrived_ = false;
	}	

	// Update rear texture
	if(new_rear_image_arrived_)
	{
		imageToTexture(texture_rear_, cur_image_rear_);
		updateTexture(texture_rear_);
		new_rear_image_arrived_ = false;
	}	

	context_->queueRender();

	if(sphere_node_)
	{
		sphere_node_->needUpdate();

//		if(!sphere_material_.isNull())
//		{
//			ROS_INFO("%d", sphere_material_->getTechnique(0)->getPass(0)->getNumTextureUnitStates());
//		}
	}
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
		setStatus(StatusProperty::Error, "Front camera image", e.what());
		ROS_ERROR("SphereDisplay::updateTexture[%s]: UnsupportedImageEncoding: %s", texture_name.c_str(), e.what());
		return;
	}
}

void SphereDisplay::imageToTexture(ROSImageTexture*& texture, const sensor_msgs::Image::ConstPtr& msg)
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
	if(sphere_material_.isNull())
	{
		ROS_ERROR("imageToTexture(): sphere_material_ is NULL.");
		return;
	}

	Ogre::Pass* pass = sphere_material_ -> getTechnique(0) -> getPass(0);
	if (!pass)
	{
		ROS_ERROR("imageToTexture(): pass is NULL.");
		return;
	}

	if (pass->getNumTextureUnitStates()<2)
	{
		ROS_ERROR("imageToTexture(): Number of texture unit states is less than 2.");
		return;
	}

	bool is_front_texture = (texture == texture_front_);

	//get unit state (0-front cam, 1-rear cam)
	int	unit_state_idx = is_front_texture ? 0 : 1; 
	Ogre::TextureUnitState* unit_state = pass -> getTextureUnitState(unit_state_idx);
	if (!unit_state)
	{
		ROS_ERROR("Failed to getTextureUnitState(%d).", unit_state_idx);
		return;
	}

	unit_state -> setTexture(texture -> getTexture());
}

void SphereDisplay::reset()
{
	//  Display::reset();
	//  clear();
}

}  // namespace rviz




#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::SphereDisplay, rviz::Display)
