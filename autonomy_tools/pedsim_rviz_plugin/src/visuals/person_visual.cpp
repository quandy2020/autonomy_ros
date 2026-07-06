/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013-2015, Timm Linder, Social Robotics Lab, University of Freiburg
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "pedsim_rviz_plugin/visuals/person_visual.hpp"

#include <vector>

#include <OgreException.h>
#include <OgrePass.h>
#include <rviz_rendering/material_manager.hpp>


namespace pedsim_rviz_plugin {

/** This helper class ensures that skeletons can be loaded from a package:// path **/
class RosPackagePathResourceLoadingListener : public Ogre::ResourceLoadingListener
{
public:
    RosPackagePathResourceLoadingListener(std::string parentPath) : _parentPath(parentPath) {
    }

    /** This event is called when a resource beings loading. */
    virtual Ogre::DataStreamPtr resourceLoading(const Ogre::String &name, const Ogre::String &group, Ogre::Resource *resource) {
      (void)group;
      (void)resource;
      std::string url = name;
      if (name.find("package://") == std::string::npos) {
        if (!name.empty() && name[0] == '/') {
          url = "file://" + name;
        } else {
          url = _parentPath + "/" + name;
        }
      }
      RVIZ_COMMON_LOG_INFO_STREAM("RosPackagePathResourceLoadingListener loading resource: " << url);
      try
      {
        resource_retriever::Retriever retriever;
        _lastResource = retriever.get(url);
        return Ogre::DataStreamPtr(new Ogre::MemoryDataStream(_lastResource.data.get(), _lastResource.size));
      }
      catch (resource_retriever::Exception& e)
      {
        RVIZ_COMMON_LOG_ERROR_STREAM("In RosPackagePathResourceLoadingListener: " << e.what());
        return Ogre::DataStreamPtr();
      }
    }

    virtual void resourceStreamOpened(const Ogre::String &name, const Ogre::String &group, Ogre::Resource *resource, Ogre::DataStreamPtr& dataStream) {
      (void)name;
      (void)group;
      (void)resource;
      (void)dataStream;
    }

    virtual bool resourceCollision(Ogre::Resource *resource, Ogre::ResourceManager *resourceManager) {
      (void)resource;
      (void)resourceManager;
      return false;
    }

private:
    const std::string _parentPath;
    resource_retriever::MemoryResource _lastResource;
};

namespace {

struct PersonMeshResource
{
    std::string path;
    std::string parent_path;
    double native_height;
    double foot_z;
    // Extra Z rotation (deg) so model walk axis aligns with parent +X (ROS forward).
    double forward_z_deg;

    bool empty() const
    {
        return path.empty();
    }
};

bool loadPersonMeshResource(const std::string & meshResource, const std::string & parentPath)
{
    Ogre::ResourceLoadingListener * newListener =
        new RosPackagePathResourceLoadingListener(parentPath);
    Ogre::ResourceLoadingListener * oldListener =
        Ogre::ResourceGroupManager::getSingleton().getLoadingListener();

    Ogre::ResourceGroupManager::getSingleton().setLoadingListener(newListener);
    const bool loadFailed = rviz_rendering::loadMeshFromResource(meshResource).isNull();
    Ogre::ResourceGroupManager::getSingleton().setLoadingListener(oldListener);

    delete newListener;
    return !loadFailed;
}

PersonMeshResource resolvePersonMeshResource()
{
    static const std::vector<PersonMeshResource> meshCandidates = {
        // Native height/foot offset measured from each mesh bounding box.
        // BoundingBoxPersonVisual depth is along +X (see rear(w,0,0) in person_visual.hpp).
        // walking.dae bind pose is elongated along +Y; rotate +Y -> +X to match the bbox.
        {"package://ros_tools/models/walking.dae", "package://ros_tools/models", 1.872074, 0.024, -90.0},
        {"package://hunav_rviz2_panel/meshes/walk.dae", "package://hunav_rviz2_panel/meshes", 1.65895, 0.0, -90.0},
        {"package://pedsim_rviz_plugin/media/male_symbol.dae", "package://pedsim_rviz_plugin/media", 1.75, 0.0, -90.0},
    };

    for (const auto & candidate : meshCandidates) {
        if (loadPersonMeshResource(candidate.path, candidate.parent_path)) {
            return candidate;
        }
        RVIZ_COMMON_LOG_WARNING_STREAM(
            "Failed to load person mesh from " << candidate.path << ", trying next candidate.");
    }

    return {};
}

void applyDefaultMaterialToEntity(Ogre::Entity * entity, const Ogre::MaterialPtr & material)
{
    if (entity == nullptr || material.isNull()) {
        return;
    }

    for (unsigned int i = 0; i < entity->getNumSubEntities(); ++i) {
        Ogre::SubEntity * subEntity = entity->getSubEntity(i);
        const std::string & materialName = subEntity->getMaterialName();
        if (materialName.empty() || materialName == "BaseWhiteNoLighting") {
            subEntity->setMaterial(material);
        }
    }
    entity->setMaterial(material);
}

void setMaterialColor(
  const Ogre::MaterialPtr & material, const Ogre::ColourValue & color)
{
    if (material.isNull()) {
        return;
    }

    Ogre::SceneBlendType blending;
    bool depth_write;
    if (color.a < 0.9998f) {
        blending = Ogre::SBT_TRANSPARENT_ALPHA;
        depth_write = false;
    } else {
        blending = Ogre::SBT_REPLACE;
        depth_write = true;
    }

    Ogre::Technique * technique = material->getTechnique(0);
    Ogre::Pass * pass = technique->getPass(0);
    pass->setAmbient(color.r * 0.5f, color.g * 0.5f, color.b * 0.5f);
    pass->setDiffuse(color.r, color.g, color.b, color.a);
    pass->setSceneBlending(blending);
    pass->setDepthWriteEnabled(depth_write);
    pass->setLightingEnabled(true);
}

}  // namespace


MeshPersonVisual::MeshPersonVisual(const PersonVisualDefaultArgs& args)
: PersonVisual(args),
  m_animationState(NULL),
  m_walkingSpeed(1.0),
  entity_(NULL),
  m_childSceneNode(NULL),
  m_meshNativeHeight(1.75),
  m_meshFootZ(0.0),
  m_userScalingFactor(1.0)
{
    // PersonVisual applies a quaternion to scale which can flip axes and hide meshes.
    m_sceneNode->setScale(Ogre::Vector3::UNIT_SCALE);

    const PersonMeshResource meshResource = resolvePersonMeshResource();
    if (meshResource.empty()) {
        RVIZ_COMMON_LOG_ERROR_STREAM(
            "Failed to load person mesh from all known resources. "
            << "Ensure ros_tools or hunav_rviz2_panel is built and sourced.");
        throw Ogre::Exception(
            Ogre::Exception::ERR_INVALIDPARAMS,
            "Person mesh resource not found",
            "MeshPersonVisual::MeshPersonVisual");
    }

    m_meshNativeHeight = meshResource.native_height;
    m_meshFootZ = meshResource.foot_z;

    // Create scene entity (names must be globally unique in Ogre SceneManager)
    static size_t count = 0;
    const std::string id = "mesh_person_visual" + std::to_string(count++);

    entity_ = m_sceneManager->createEntity(id, meshResource.path);

    static size_t mesh_node_counter = 0;
    const std::string mesh_node_name = kPersonMeshNodePrefix + std::to_string(mesh_node_counter++);
    m_childSceneNode = m_sceneNode->createChildSceneNode(mesh_node_name);
    m_childSceneNode->attachObject(entity_);

    // set up animation
    setAnimationState("");

    // set up material
    const std::string materialName = id + "Material";
    Ogre::MaterialPtr default_material =
        rviz_rendering::MaterialManager::createMaterialWithLighting(materialName);
    default_material->getTechnique(0)->setAmbient(0.5f, 0.5f, 0.5f);
    materials_.insert(default_material);
    applyDefaultMaterialToEntity(entity_, default_material);

    // Match BoundingBoxPersonVisual: person forward is +X in the parent scene node.
    // Z-up humanoid meshes (walking.dae, walk.dae) walk along model +Y; rotate onto +X.
    Ogre::Quaternion alignForward = Ogre::Quaternion::IDENTITY;
    if (std::abs(meshResource.forward_z_deg) > 1e-6) {
        alignForward.FromAngleAxis(
            Ogre::Degree(meshResource.forward_z_deg), Ogre::Vector3::UNIT_Z);
    }
    m_childSceneNode->setOrientation(alignForward);

    applyMeshScale(m_userScalingFactor);
    m_childSceneNode->setVisible(true);
}

MeshPersonVisual::~MeshPersonVisual() {
    if (entity_ != nullptr) {
        if (m_childSceneNode != nullptr) {
            m_childSceneNode->detachObject(entity_);
        }
        m_sceneManager->destroyEntity(entity_);
        entity_ = nullptr;
    }

    // destroy all the materials we've created
    std::set<Ogre::MaterialPtr>::iterator it;
    for ( it = materials_.begin(); it!=materials_.end(); it++ )
    {
        Ogre::MaterialPtr material = *it;
        if (!material.isNull())
        {
          const Ogre::String& group = material->getGroup();
          if (Ogre::MaterialManager::getSingleton().resourceExists(material->getName(), group))
          {
            material->unload();
            Ogre::MaterialManager::getSingleton().remove(material->getName(), group);
          }
        }
    }
    materials_.clear();
    if (m_childSceneNode != nullptr) {
        m_sceneNode->removeChild(m_childSceneNode);
        m_sceneManager->destroySceneNode(m_childSceneNode);
        m_childSceneNode = nullptr;
    }
}

void MeshPersonVisual::applyMeshScale(double scalingFactor)
{
    if (m_childSceneNode == nullptr || m_meshNativeHeight <= 0.0) {
        return;
    }

    m_userScalingFactor = scalingFactor;
    const double scaleFactor = (getHeight() / m_meshNativeHeight) * m_userScalingFactor;
    m_childSceneNode->setScale(Ogre::Vector3(scaleFactor, scaleFactor, scaleFactor));

    // Parent node is offset to height/2; shift child down so feet sit on the ground plane.
    m_childSceneNode->setPosition(
        Ogre::Vector3(0, 0, -getHeight() * 0.5 + m_meshFootZ * scaleFactor));
}

void MeshPersonVisual::setColor(const Ogre::ColourValue& c) {
    std::set<Ogre::MaterialPtr>::iterator it;
    for (it = materials_.begin(); it != materials_.end(); ++it) {
      setMaterialColor(*it, c);
    }
}

void MeshPersonVisual::setAnimationState(const std::string& nameOfAnimationState) {
    Ogre::AnimationStateSet *animationStates = entity_->getAllAnimationStates();
    if(animationStates != NULL)
    {
      Ogre::AnimationStateIterator animationsIterator = animationStates->getAnimationStateIterator();
      while (animationsIterator.hasMoreElements())
      {
        Ogre::AnimationState *animationState = animationsIterator.getNext();
        if(animationState->getAnimationName() == nameOfAnimationState || nameOfAnimationState.empty()) {
          animationState->setLoop(true);
          animationState->setEnabled(true);
          m_animationState = animationState;
          return;
        }    
      }

      // Not found. Set first animation state then.
      RVIZ_COMMON_LOG_WARNING_STREAM("Person mesh animation state " << nameOfAnimationState << " does not exist in mesh!");
      setAnimationState("");
    }
}

void MeshPersonVisual::setWalkingSpeed(float walkingSpeed) {
    m_walkingSpeed = walkingSpeed;
}


void MeshPersonVisual::update(float deltaTime) {
    if(m_animationState) {
        m_animationState->addTime(0.7 * deltaTime * m_walkingSpeed);
    }
}


} // end of namespace pedsim_rviz_plugin
