#ifndef AUTONOMY_MSGS__GRAPH_DISPLAY_HPP_
#define AUTONOMY_MSGS__GRAPH_DISPLAY_HPP_

#include <memory>
#include <unordered_map>
#include <vector>

#include <OgreManualObject.h>
#include <autonomy_msgs/msg/graph.hpp>
#include <autonomy_msgs/msg/graph_edge.hpp>
#include <autonomy_msgs/msg/graph_face.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/shape.hpp>

namespace autonomy_msgs
{

class GraphDisplay : public rviz_common::RosTopicDisplay<autonomy_msgs::msg::Graph>
{
  Q_OBJECT

public:
  GraphDisplay();
  ~GraphDisplay() override;

protected:
  void onInitialize() override;
  void reset() override;
  void processMessage(autonomy_msgs::msg::Graph::ConstSharedPtr msg) override;

private:
  struct EdgeVisual
  {
    std::vector<std::shared_ptr<rviz_rendering::BillboardLine>> lines;
    std::vector<std::shared_ptr<rviz_rendering::Arrow>> arrows;
  };

  struct FaceVisual
  {
    Ogre::ManualObject * fill{nullptr};
    std::shared_ptr<rviz_rendering::BillboardLine> outline;
  };

  void clearVisuals();
  void rebuildVisuals();
  void addEdgeVisual(
    const Ogre::Vector3 & from,
    const Ogre::Vector3 & to,
    const autonomy_msgs::msg::GraphEdge & edge);
  void addFaceVisual(
    const autonomy_msgs::msg::GraphFace & face,
    const std::unordered_map<uint32_t, Ogre::Vector3> & node_positions);
  void addDirectionArrow(
    const Ogre::Vector3 & from,
    const Ogre::Vector3 & to,
    const Ogre::Vector3 & direction,
    float line_width,
    const Ogre::ColourValue & color,
    EdgeVisual & visual);
  void addLinkShapeMarker(
    const Ogre::Vector3 & from,
    const Ogre::Vector3 & to,
    uint8_t link_shape,
    float line_width,
    const Ogre::ColourValue & color,
    EdgeVisual & visual);

  std::vector<Ogre::Vector3> resolveFaceVertices(
    const autonomy_msgs::msg::GraphFace & face,
    const std::unordered_map<uint32_t, Ogre::Vector3> & node_positions) const;
  Ogre::Vector3 perpendicular(const Ogre::Vector3 & direction) const;

  bool shouldShowEdgeType(uint8_t type) const;
  bool shouldShowLinkShape(uint8_t link_shape) const;
  bool shouldShowFaceShape(uint8_t shape) const;
  bool shouldRenderEdge(uint8_t message_link_shape, uint8_t & render_shape) const;
  bool shouldRenderFace(uint8_t message_face_shape, uint8_t & render_shape) const;
  float resolveEdgeLineWidth(uint8_t type, uint8_t link_shape) const;
  Ogre::ColourValue resolveEdgeColor(
    const autonomy_msgs::msg::GraphEdge & edge, uint8_t render_link_shape) const;
  Ogre::ColourValue toOgreColor(const std_msgs::msg::ColorRGBA & color) const;
  Ogre::ColourValue resolveNodeColor(const autonomy_msgs::msg::GraphNode & node) const;
  bool isUnsetColor(const std_msgs::msg::ColorRGBA & color) const;
  std::vector<Ogre::Vector3> resolveEdgePolyline(
    const autonomy_msgs::msg::GraphEdge & edge,
    const Ogre::Vector3 & from,
    const Ogre::Vector3 & to) const;
  float resolveNodeScale(const autonomy_msgs::msg::GraphNode & node) const;

  Ogre::SceneNode * graph_node_{nullptr};
  autonomy_msgs::msg::Graph::ConstSharedPtr last_graph_msg_;
  std::vector<std::shared_ptr<rviz_rendering::Shape>> node_shapes_;
  std::vector<EdgeVisual> edge_visuals_;
  std::vector<FaceVisual> face_visuals_;
  uint64_t manual_object_counter_{0};

  rviz_common::properties::FloatProperty * default_node_scale_property_{nullptr};
  rviz_common::properties::FloatProperty * default_line_width_property_{nullptr};
  rviz_common::properties::BoolProperty * show_nodes_property_{nullptr};
  rviz_common::properties::BoolProperty * show_edges_property_{nullptr};
  rviz_common::properties::BoolProperty * show_faces_property_{nullptr};
  rviz_common::properties::FloatProperty * face_alpha_scale_property_{nullptr};
  rviz_common::properties::Property * edge_types_property_{nullptr};
  rviz_common::properties::BoolProperty * show_undirected_property_{nullptr};
  rviz_common::properties::BoolProperty * show_directed_property_{nullptr};
  rviz_common::properties::BoolProperty * show_bidirectional_property_{nullptr};
  rviz_common::properties::BoolProperty * show_virtual_property_{nullptr};
  rviz_common::properties::FloatProperty * undirected_width_scale_property_{nullptr};
  rviz_common::properties::FloatProperty * directed_width_scale_property_{nullptr};
  rviz_common::properties::FloatProperty * bidirectional_width_scale_property_{nullptr};
  rviz_common::properties::FloatProperty * virtual_width_scale_property_{nullptr};
  rviz_common::properties::Property * link_shapes_property_{nullptr};
  rviz_common::properties::BoolProperty * show_line_links_property_{nullptr};
  rviz_common::properties::BoolProperty * show_triangle_links_property_{nullptr};
  rviz_common::properties::BoolProperty * show_rectangle_links_property_{nullptr};
  rviz_common::properties::BoolProperty * show_polygon_links_property_{nullptr};
  rviz_common::properties::FloatProperty * triangle_link_width_scale_property_{nullptr};
  rviz_common::properties::FloatProperty * rectangle_link_width_scale_property_{nullptr};
  rviz_common::properties::FloatProperty * polygon_link_width_scale_property_{nullptr};
  rviz_common::properties::Property * face_shapes_property_{nullptr};
  rviz_common::properties::BoolProperty * show_triangle_faces_property_{nullptr};
  rviz_common::properties::BoolProperty * show_rectangle_faces_property_{nullptr};
  rviz_common::properties::BoolProperty * show_polygon_faces_property_{nullptr};

private Q_SLOTS:
  void updateStyles();
};

}  // namespace autonomy_msgs

#endif  // AUTONOMY_MSGS__GRAPH_DISPLAY_HPP_
