#ifndef PEDSIM_RVIZ_PLUGIN__SCENE_NODE_PTR_HPP_
#define PEDSIM_RVIZ_PLUGIN__SCENE_NODE_PTR_HPP_

#include <memory>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

namespace pedsim_rviz_plugin
{

using SceneNodePtr = std::shared_ptr<Ogre::SceneNode>;

inline SceneNodePtr make_scene_node_ptr(Ogre::SceneManager * scene_manager, Ogre::SceneNode * node)
{
  return SceneNodePtr(node, [scene_manager](Ogre::SceneNode * n) {
      if (n != nullptr && !n->getName().empty()) {
        scene_manager->destroySceneNode(n->getName());
      }
    });
}

inline SceneNodePtr create_child_scene_node_ptr(
  Ogre::SceneManager * scene_manager,
  Ogre::SceneNode * parent)
{
  return make_scene_node_ptr(scene_manager, parent->createChildSceneNode());
}

}  // namespace pedsim_rviz_plugin

#endif  // PEDSIM_RVIZ_PLUGIN__SCENE_NODE_PTR_HPP_
