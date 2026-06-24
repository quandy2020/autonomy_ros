#include "autonomy_msgs/graph_display.hpp"

#include <algorithm>
#include <sstream>

#include <OgreSceneNode.h>

namespace autonomy_msgs
{

GraphDisplay::GraphDisplay()
{
  qos_profile = rclcpp::QoS(1).transient_local().reliable();
}

GraphDisplay::~GraphDisplay()
{
  clearVisuals();
}

void GraphDisplay::onInitialize()
{
  RosTopicDisplay::onInitialize();

  default_node_scale_property_ = new rviz_common::properties::FloatProperty(
    "Default node scale", 0.08f, "Sphere radius when GraphNode.scale is 0", this,
    SLOT(updateStyles()));
  default_line_width_property_ = new rviz_common::properties::FloatProperty(
    "Line width", 0.03f, "Base edge line width", this, SLOT(updateStyles()));
  show_nodes_property_ = new rviz_common::properties::BoolProperty(
    "Show nodes", true, "Render graph nodes", this, SLOT(updateStyles()));
  show_edges_property_ = new rviz_common::properties::BoolProperty(
    "Show edges", true, "Render graph edges", this, SLOT(updateStyles()));
  show_faces_property_ = new rviz_common::properties::BoolProperty(
    "Show faces", true, "Render filled triangle/rectangle/polygon facets", this,
    SLOT(updateStyles()));
  face_alpha_scale_property_ = new rviz_common::properties::FloatProperty(
    "Face alpha scale", 1.0f, "Multiplier for GraphFace.alpha", this, SLOT(updateStyles()));

  edge_types_property_ = new rviz_common::properties::Property(
    "Edge types", QVariant(), "Visibility and width per GraphEdge.type", this,
    SLOT(updateStyles()));

  show_undirected_property_ = new rviz_common::properties::BoolProperty(
    "Undirected", true, "Show TYPE_UNDIRECTED edges", edge_types_property_,
    SLOT(updateStyles()), this);
  show_directed_property_ = new rviz_common::properties::BoolProperty(
    "Directed", true, "Show TYPE_DIRECTED edges (line + arrow)", edge_types_property_,
    SLOT(updateStyles()), this);
  show_bidirectional_property_ = new rviz_common::properties::BoolProperty(
    "Bidirectional", true, "Show TYPE_BIDIRECTIONAL edges", edge_types_property_,
    SLOT(updateStyles()), this);
  show_virtual_property_ = new rviz_common::properties::BoolProperty(
    "Virtual", true, "Show TYPE_VIRTUAL edges", edge_types_property_,
    SLOT(updateStyles()), this);

  undirected_width_scale_property_ = new rviz_common::properties::FloatProperty(
    "Undirected width scale", 1.0f, "", edge_types_property_, SLOT(updateStyles()), this);
  directed_width_scale_property_ = new rviz_common::properties::FloatProperty(
    "Directed width scale", 1.0f, "", edge_types_property_, SLOT(updateStyles()), this);
  bidirectional_width_scale_property_ = new rviz_common::properties::FloatProperty(
    "Bidirectional width scale", 1.0f, "", edge_types_property_, SLOT(updateStyles()), this);
  virtual_width_scale_property_ = new rviz_common::properties::FloatProperty(
    "Virtual width scale", 0.5f, "", edge_types_property_, SLOT(updateStyles()), this);

  link_shapes_property_ = new rviz_common::properties::Property(
    "Link shapes", QVariant(),
    "When one shape is selected, all edges render with that style. "
    "When multiple are selected, edges are filtered by GraphEdge.link_shape.",
    this, SLOT(updateStyles()));

  show_line_links_property_ = new rviz_common::properties::BoolProperty(
    "Line", true, "Show LINK_SHAPE_LINE edges", link_shapes_property_, SLOT(updateStyles()),
    this);
  show_triangle_links_property_ = new rviz_common::properties::BoolProperty(
    "Triangle", true, "Show LINK_SHAPE_TRIANGLE edges (line + triangle marker)",
    link_shapes_property_, SLOT(updateStyles()), this);
  show_rectangle_links_property_ = new rviz_common::properties::BoolProperty(
    "Rectangle", true, "Show LINK_SHAPE_RECTANGLE edges (line + rectangle marker)",
    link_shapes_property_, SLOT(updateStyles()), this);
  show_polygon_links_property_ = new rviz_common::properties::BoolProperty(
    "Polygon", true, "Show LINK_SHAPE_POLYGON edges (double parallel lines)",
    link_shapes_property_, SLOT(updateStyles()), this);

  triangle_link_width_scale_property_ = new rviz_common::properties::FloatProperty(
    "Triangle width scale", 1.0f, "", link_shapes_property_, SLOT(updateStyles()), this);
  rectangle_link_width_scale_property_ = new rviz_common::properties::FloatProperty(
    "Rectangle width scale", 1.1f, "", link_shapes_property_, SLOT(updateStyles()), this);
  polygon_link_width_scale_property_ = new rviz_common::properties::FloatProperty(
    "Polygon width scale", 0.9f, "", link_shapes_property_, SLOT(updateStyles()), this);

  face_shapes_property_ = new rviz_common::properties::Property(
    "Face shapes", QVariant(),
    "When one shape is selected, all faces render with that style. "
    "When multiple are selected, faces are filtered by GraphFace.shape.",
    this, SLOT(updateStyles()));

  show_triangle_faces_property_ = new rviz_common::properties::BoolProperty(
    "Triangle", true, "Show triangle facets", face_shapes_property_, SLOT(updateStyles()), this);
  show_rectangle_faces_property_ = new rviz_common::properties::BoolProperty(
    "Rectangle", true, "Show rectangle facets", face_shapes_property_, SLOT(updateStyles()), this);
  show_polygon_faces_property_ = new rviz_common::properties::BoolProperty(
    "Polygon", true, "Show polygon facets", face_shapes_property_, SLOT(updateStyles()), this);

  graph_node_ = scene_node_->createChildSceneNode();
}

void GraphDisplay::reset()
{
  RosTopicDisplay::reset();
  last_graph_msg_.reset();
  clearVisuals();
}

void GraphDisplay::clearVisuals()
{
  node_shapes_.clear();
  edge_visuals_.clear();

  for (auto & face : face_visuals_) {
    if (face.fill != nullptr) {
      graph_node_->detachObject(face.fill);
      context_->getSceneManager()->destroyManualObject(face.fill);
      face.fill = nullptr;
    }
  }
  face_visuals_.clear();

  if (graph_node_ != nullptr) {
    graph_node_->removeAndDestroyAllChildren();
  }
}

Ogre::ColourValue GraphDisplay::toOgreColor(const std_msgs::msg::ColorRGBA & color) const
{
  return Ogre::ColourValue(color.r, color.g, color.b, color.a);
}

float GraphDisplay::resolveNodeScale(const autonomy_msgs::msg::GraphNode & node) const
{
  float scale = node.scale;
  if (scale <= 0.0f) {
    scale = default_node_scale_property_->getFloat();
  }
  using autonomy_msgs::msg::GraphNode;
  if ((node.flags & GraphNode::FLAG_START) || (node.flags & GraphNode::FLAG_GOAL)) {
    scale *= 1.35f;
  }
  return scale;
}

bool GraphDisplay::isUnsetColor(const std_msgs::msg::ColorRGBA & color) const
{
  return color.a < 1e-4f;
}

Ogre::ColourValue GraphDisplay::resolveNodeColor(
  const autonomy_msgs::msg::GraphNode & node) const
{
  if (!isUnsetColor(node.color)) {
    return toOgreColor(node.color);
  }

  using autonomy_msgs::msg::GraphNode;
  if (node.flags & GraphNode::FLAG_START) {
    return Ogre::ColourValue(0.15f, 0.85f, 0.25f, 1.0f);
  }
  if (node.flags & GraphNode::FLAG_GOAL) {
    return Ogre::ColourValue(0.2f, 0.45f, 0.95f, 1.0f);
  }
  if (node.flags & GraphNode::FLAG_OBSTACLE_VERTEX) {
    return Ogre::ColourValue(0.95f, 0.75f, 0.1f, 1.0f);
  }
  if (node.flags & GraphNode::FLAG_WAYPOINT) {
    return Ogre::ColourValue(0.85f, 0.35f, 0.85f, 1.0f);
  }
  return Ogre::ColourValue(0.95f, 0.15f, 0.15f, 1.0f);
}

std::vector<Ogre::Vector3> GraphDisplay::resolveEdgePolyline(
  const autonomy_msgs::msg::GraphEdge & edge,
  const Ogre::Vector3 & from,
  const Ogre::Vector3 & to) const
{
  if (edge.path.empty()) {
    return {from, to};
  }

  std::vector<Ogre::Vector3> points;
  points.reserve(edge.path.size());
  for (const auto & point : edge.path) {
    points.emplace_back(point.x, point.y, point.z);
  }
  return points;
}

bool GraphDisplay::shouldShowEdgeType(uint8_t type) const
{
  using autonomy_msgs::msg::GraphEdge;
  switch (type) {
    case GraphEdge::TYPE_DIRECTED:
      return show_directed_property_->getBool();
    case GraphEdge::TYPE_BIDIRECTIONAL:
      return show_bidirectional_property_->getBool();
    case GraphEdge::TYPE_VIRTUAL:
      return show_virtual_property_->getBool();
    default:
      return show_undirected_property_->getBool();
  }
}

bool GraphDisplay::shouldShowLinkShape(uint8_t link_shape) const
{
  using autonomy_msgs::msg::GraphEdge;
  switch (link_shape) {
    case GraphEdge::LINK_SHAPE_TRIANGLE:
      return show_triangle_links_property_->getBool();
    case GraphEdge::LINK_SHAPE_RECTANGLE:
      return show_rectangle_links_property_->getBool();
    case GraphEdge::LINK_SHAPE_POLYGON:
      return show_polygon_links_property_->getBool();
    default:
      return show_line_links_property_->getBool();
  }
}

bool GraphDisplay::shouldShowFaceShape(uint8_t shape) const
{
  using autonomy_msgs::msg::GraphFace;
  switch (shape) {
    case GraphFace::SHAPE_RECTANGLE:
      return show_rectangle_faces_property_->getBool();
    case GraphFace::SHAPE_POLYGON:
      return show_polygon_faces_property_->getBool();
    default:
      return show_triangle_faces_property_->getBool();
  }
}

bool GraphDisplay::shouldRenderEdge(uint8_t message_link_shape, uint8_t & render_shape) const
{
  using autonomy_msgs::msg::GraphEdge;

  const bool show_line = show_line_links_property_->getBool();
  const bool show_triangle = show_triangle_links_property_->getBool();
  const bool show_rectangle = show_rectangle_links_property_->getBool();
  const bool show_polygon = show_polygon_links_property_->getBool();

  const int selected_count =
    static_cast<int>(show_line) + static_cast<int>(show_triangle) +
    static_cast<int>(show_rectangle) + static_cast<int>(show_polygon);
  if (selected_count == 0) {
    return false;
  }
  if (selected_count == 1) {
    if (show_line) {
      render_shape = GraphEdge::LINK_SHAPE_LINE;
    } else if (show_triangle) {
      render_shape = GraphEdge::LINK_SHAPE_TRIANGLE;
    } else if (show_rectangle) {
      render_shape = GraphEdge::LINK_SHAPE_RECTANGLE;
    } else {
      render_shape = GraphEdge::LINK_SHAPE_POLYGON;
    }
    return true;
  }

  if (!shouldShowLinkShape(message_link_shape)) {
    return false;
  }
  render_shape = message_link_shape;
  return true;
}

bool GraphDisplay::shouldRenderFace(uint8_t message_face_shape, uint8_t & render_shape) const
{
  using autonomy_msgs::msg::GraphFace;

  const bool show_triangle = show_triangle_faces_property_->getBool();
  const bool show_rectangle = show_rectangle_faces_property_->getBool();
  const bool show_polygon = show_polygon_faces_property_->getBool();

  const int selected_count =
    static_cast<int>(show_triangle) + static_cast<int>(show_rectangle) +
    static_cast<int>(show_polygon);
  if (selected_count == 0) {
    return false;
  }
  if (selected_count == 1) {
    if (show_triangle) {
      render_shape = GraphFace::SHAPE_TRIANGLE;
    } else if (show_rectangle) {
      render_shape = GraphFace::SHAPE_RECTANGLE;
    } else {
      render_shape = GraphFace::SHAPE_POLYGON;
    }
    return true;
  }

  if (!shouldShowFaceShape(message_face_shape)) {
    return false;
  }
  render_shape = message_face_shape;
  return true;
}

float GraphDisplay::resolveEdgeLineWidth(uint8_t type, uint8_t link_shape) const
{
  using autonomy_msgs::msg::GraphEdge;
  const float base = default_line_width_property_->getFloat();
  float type_scale = undirected_width_scale_property_->getFloat();
  switch (type) {
    case GraphEdge::TYPE_DIRECTED:
      type_scale = directed_width_scale_property_->getFloat();
      break;
    case GraphEdge::TYPE_BIDIRECTIONAL:
      type_scale = bidirectional_width_scale_property_->getFloat();
      break;
    case GraphEdge::TYPE_VIRTUAL:
      type_scale = virtual_width_scale_property_->getFloat();
      break;
    default:
      break;
  }

  float shape_scale = 1.0f;
  switch (link_shape) {
    case GraphEdge::LINK_SHAPE_TRIANGLE:
      shape_scale = triangle_link_width_scale_property_->getFloat();
      break;
    case GraphEdge::LINK_SHAPE_RECTANGLE:
      shape_scale = rectangle_link_width_scale_property_->getFloat();
      break;
    case GraphEdge::LINK_SHAPE_POLYGON:
      shape_scale = polygon_link_width_scale_property_->getFloat();
      break;
    default:
      break;
  }
  return base * type_scale * shape_scale;
}

Ogre::ColourValue GraphDisplay::resolveEdgeColor(
  const autonomy_msgs::msg::GraphEdge & edge,
  const uint8_t render_link_shape) const
{
  using autonomy_msgs::msg::GraphEdge;
  auto color = toOgreColor(edge.color);
  if (edge.type == GraphEdge::TYPE_VIRTUAL) {
    color.a *= 0.45f;
  }
  switch (render_link_shape) {
    case GraphEdge::LINK_SHAPE_TRIANGLE:
      color.b = std::min(1.0f, color.b + 0.15f);
      break;
    case GraphEdge::LINK_SHAPE_RECTANGLE:
      color.r = std::min(1.0f, color.r + 0.12f);
      break;
    case GraphEdge::LINK_SHAPE_POLYGON:
      color.g = std::min(1.0f, color.g + 0.08f);
      color.b = std::min(1.0f, color.b + 0.12f);
      break;
    default:
      break;
  }
  return color;
}

Ogre::Vector3 GraphDisplay::perpendicular(const Ogre::Vector3 & direction) const
{
  Ogre::Vector3 dir = direction;
  if (dir.squaredLength() < 1e-8f) {
    return Ogre::Vector3::UNIT_Z;
  }
  dir.normalise();
  Ogre::Vector3 perp = dir.crossProduct(Ogre::Vector3::UNIT_Z);
  if (perp.squaredLength() < 1e-8f) {
    perp = dir.crossProduct(Ogre::Vector3::UNIT_Y);
  }
  perp.normalise();
  return perp;
}

void GraphDisplay::processMessage(autonomy_msgs::msg::Graph::ConstSharedPtr msg)
{
  last_graph_msg_ = msg;
  rebuildVisuals();
}

void GraphDisplay::addDirectionArrow(
  const Ogre::Vector3 & from,
  const Ogre::Vector3 & to,
  const Ogre::Vector3 & direction,
  const float line_width,
  const Ogre::ColourValue & color,
  EdgeVisual & visual)
{
  Ogre::Vector3 dir = direction;
  if (dir.squaredLength() < 1e-8f) {
    dir = to - from;
  }
  if (dir.squaredLength() < 1e-8f) {
    return;
  }
  dir.normalise();

  const float head_len = std::max(0.02f, line_width * 4.0f);
  const float head_diam = std::max(0.02f, line_width * 2.5f);
  auto arrow = std::make_shared<rviz_rendering::Arrow>(
    context_->getSceneManager(), graph_node_, 0.0f, line_width * 0.5f, head_len, head_diam);

  const Ogre::Vector3 tip = to - dir * head_len * 0.5f;
  arrow->setPosition(tip);
  arrow->setDirection(dir);
  arrow->setColor(color);
  visual.arrows.push_back(arrow);
}

void GraphDisplay::addLinkShapeMarker(
  const Ogre::Vector3 & from,
  const Ogre::Vector3 & to,
  const uint8_t link_shape,
  const float line_width,
  const Ogre::ColourValue & color,
  EdgeVisual & visual)
{
  using autonomy_msgs::msg::GraphEdge;
  const Ogre::Vector3 mid = (from + to) * 0.5f;
  const Ogre::Vector3 direction = to - from;
  const float edge_len = direction.length();
  const Ogre::Vector3 perp = perpendicular(direction);
  const float marker_size = std::max(0.05f, std::min(edge_len * 0.35f, line_width * 8.0f));

  auto add_closed_loop = [&](const std::vector<Ogre::Vector3> & points) {
    if (points.size() < 3) {
      return;
    }
    auto loop = std::make_shared<rviz_rendering::BillboardLine>(
      context_->getSceneManager(), graph_node_);
    loop->setNumLines(1);
    loop->setMaxPointsPerLine(static_cast<int>(points.size() + 1));
    loop->setLineWidth(line_width * 1.2f);
    for (const auto & point : points) {
      loop->addPoint(point);
    }
    loop->addPoint(points.front());
    loop->setColor(color.r, color.g, color.b, color.a);
    visual.lines.push_back(loop);
  };

  if (link_shape == GraphEdge::LINK_SHAPE_TRIANGLE) {
    const Ogre::Vector3 p0 = mid + perp * marker_size;
    const Ogre::Vector3 p1 = mid - perp * marker_size * 0.5f + Ogre::Vector3::UNIT_Z * marker_size * 0.75f;
    const Ogre::Vector3 p2 = mid - perp * marker_size * 0.5f - Ogre::Vector3::UNIT_Z * marker_size * 0.75f;
    add_closed_loop({p0, p1, p2});
    return;
  }

  if (link_shape == GraphEdge::LINK_SHAPE_RECTANGLE) {
    const Ogre::Vector3 z = Ogre::Vector3::UNIT_Z * marker_size * 0.6f;
    const Ogre::Vector3 p = perp * marker_size * 0.6f;
    add_closed_loop({
      mid - p - z,
      mid + p - z,
      mid + p + z,
      mid - p + z,
    });
    return;
  }

  if (link_shape == GraphEdge::LINK_SHAPE_POLYGON) {
    const float offset = std::max(0.02f, line_width * 2.5f);
    Ogre::Vector3 dir = direction;
    if (dir.squaredLength() > 1e-8f) {
      dir.normalise();
    } else {
      dir = Ogre::Vector3::UNIT_X;
    }

    auto parallel = std::make_shared<rviz_rendering::BillboardLine>(
      context_->getSceneManager(), graph_node_);
    parallel->setNumLines(1);
    parallel->setMaxPointsPerLine(2);
    parallel->setLineWidth(line_width * 0.85f);
    parallel->addPoint(from + perp * offset);
    parallel->addPoint(to + perp * offset);
    parallel->setColor(color.r, color.g, color.b, color.a * 0.85f);
    visual.lines.push_back(parallel);

    auto parallel2 = std::make_shared<rviz_rendering::BillboardLine>(
      context_->getSceneManager(), graph_node_);
    parallel2->setNumLines(1);
    parallel2->setMaxPointsPerLine(2);
    parallel2->setLineWidth(line_width * 0.85f);
    parallel2->addPoint(from - perp * offset);
    parallel2->addPoint(to - perp * offset);
    parallel2->setColor(color.r, color.g, color.b, color.a * 0.85f);
    visual.lines.push_back(parallel2);
  }
}

void GraphDisplay::addEdgeVisual(
  const Ogre::Vector3 & from,
  const Ogre::Vector3 & to,
  const autonomy_msgs::msg::GraphEdge & edge)
{
  using autonomy_msgs::msg::GraphEdge;

  if (!shouldShowEdgeType(edge.type)) {
    return;
  }

  uint8_t render_link_shape = GraphEdge::LINK_SHAPE_LINE;
  if (!shouldRenderEdge(edge.link_shape, render_link_shape)) {
    return;
  }

  const float line_width = resolveEdgeLineWidth(edge.type, render_link_shape);
  const auto edge_color = resolveEdgeColor(edge, render_link_shape);
  const auto polyline = resolveEdgePolyline(edge, from, to);
  if (polyline.size() < 2) {
    return;
  }
  const Ogre::Vector3 direction = polyline.back() - polyline[polyline.size() - 2];

  EdgeVisual visual;
  auto line = std::make_shared<rviz_rendering::BillboardLine>(
    context_->getSceneManager(), graph_node_);
  line->setNumLines(1);
  line->setMaxPointsPerLine(static_cast<int>(polyline.size()));
  line->setLineWidth(line_width);
  for (const auto & point : polyline) {
    line->addPoint(point);
  }
  line->setColor(edge_color.r, edge_color.g, edge_color.b, edge_color.a);
  visual.lines.push_back(line);

  if (render_link_shape != GraphEdge::LINK_SHAPE_LINE && edge.path.empty()) {
    addLinkShapeMarker(from, to, render_link_shape, line_width, edge_color, visual);
  }

  if (edge.type == GraphEdge::TYPE_DIRECTED) {
    addDirectionArrow(
      polyline[polyline.size() - 2], polyline.back(), direction, line_width, edge_color, visual);
  } else if (edge.type == GraphEdge::TYPE_BIDIRECTIONAL) {
    addDirectionArrow(
      polyline[polyline.size() - 2], polyline.back(), direction, line_width, edge_color, visual);
    addDirectionArrow(
      polyline[1], polyline.front(), polyline.front() - polyline[1], line_width, edge_color, visual);
  }

  edge_visuals_.push_back(std::move(visual));
}

std::vector<Ogre::Vector3> GraphDisplay::resolveFaceVertices(
  const autonomy_msgs::msg::GraphFace & face,
  const std::unordered_map<uint32_t, Ogre::Vector3> & node_positions) const
{
  std::vector<Ogre::Vector3> vertices;
  if (!face.points.empty()) {
    vertices.reserve(face.points.size());
    for (const auto & point : face.points) {
      vertices.emplace_back(point.x, point.y, point.z);
    }
    return vertices;
  }

  vertices.reserve(face.node_ids.size());
  for (const auto node_id : face.node_ids) {
    const auto it = node_positions.find(node_id);
    if (it == node_positions.end()) {
      return {};
    }
    vertices.push_back(it->second);
  }
  return vertices;
}

void GraphDisplay::addFaceVisual(
  const autonomy_msgs::msg::GraphFace & face,
  const std::unordered_map<uint32_t, Ogre::Vector3> & node_positions)
{
  using autonomy_msgs::msg::GraphFace;

  uint8_t render_face_shape = GraphFace::SHAPE_TRIANGLE;
  if (!shouldRenderFace(face.shape, render_face_shape)) {
    return;
  }

  const auto vertices = resolveFaceVertices(face, node_positions);
  if (vertices.size() < 3) {
    return;
  }

  const auto color = toOgreColor(face.color);
  const float alpha = std::clamp(face.alpha * face_alpha_scale_property_->getFloat(), 0.0f, 1.0f);

  std::ostringstream name_stream;
  name_stream << "GraphFace" << manual_object_counter_++;
  Ogre::ManualObject * manual = context_->getSceneManager()->createManualObject(
    name_stream.str());
  manual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
  manual->estimateVertexCount(vertices.size() * 3);
  manual->estimateIndexCount(vertices.size() * 3);

  const auto emit_triangle = [&](const Ogre::Vector3 & a, const Ogre::Vector3 & b,
      const Ogre::Vector3 & c) {
    manual->position(a);
    manual->colour(color.r, color.g, color.b, alpha);
    manual->position(b);
    manual->colour(color.r, color.g, color.b, alpha);
    manual->position(c);
    manual->colour(color.r, color.g, color.b, alpha);
  };

  if (render_face_shape == GraphFace::SHAPE_TRIANGLE && vertices.size() >= 3) {
    emit_triangle(vertices[0], vertices[1], vertices[2]);
  } else if (render_face_shape == GraphFace::SHAPE_RECTANGLE && vertices.size() >= 4) {
    emit_triangle(vertices[0], vertices[1], vertices[2]);
    emit_triangle(vertices[0], vertices[2], vertices[3]);
  } else {
    for (size_t i = 1; i + 1 < vertices.size(); ++i) {
      emit_triangle(vertices[0], vertices[i], vertices[i + 1]);
    }
  }
  manual->end();
  manual->setCastShadows(false);
  graph_node_->attachObject(manual);

  FaceVisual visual;
  visual.fill = manual;

  auto outline = std::make_shared<rviz_rendering::BillboardLine>(
    context_->getSceneManager(), graph_node_);
  outline->setNumLines(1);
  outline->setMaxPointsPerLine(static_cast<int>(vertices.size() + 1));
  outline->setLineWidth(default_line_width_property_->getFloat() * 0.6f);
  for (const auto & vertex : vertices) {
    outline->addPoint(vertex);
  }
  outline->addPoint(vertices.front());
  outline->setColor(color.r, color.g, color.b, std::min(1.0f, alpha + 0.25f));
  visual.outline = outline;

  face_visuals_.push_back(std::move(visual));
}

void GraphDisplay::rebuildVisuals()
{
  clearVisuals();
  if (last_graph_msg_ == nullptr) {
    return;
  }

  const auto & msg = last_graph_msg_;
  std::unordered_map<uint32_t, Ogre::Vector3> node_positions;
  node_positions.reserve(msg->nodes.size());
  for (const auto & node : msg->nodes) {
    node_positions.emplace(
      node.id,
      Ogre::Vector3(node.position.x, node.position.y, node.position.z));
  }

  if (show_faces_property_->getBool() &&
    last_graph_msg_->kind != autonomy_msgs::msg::Graph::KIND_VISIBILITY)
  {
    for (const auto & face : msg->faces) {
      addFaceVisual(face, node_positions);
    }
  }

  if (show_nodes_property_->getBool()) {
    for (const auto & node : msg->nodes) {
      auto shape = std::make_shared<rviz_rendering::Shape>(
        rviz_rendering::Shape::Sphere, context_->getSceneManager(), graph_node_);
      const float scale = resolveNodeScale(node);
      shape->setScale(Ogre::Vector3(scale, scale, scale));
      shape->setPosition(Ogre::Vector3(node.position.x, node.position.y, node.position.z));
      shape->setColor(resolveNodeColor(node));
      node_shapes_.push_back(shape);
    }
  }

  if (show_edges_property_->getBool()) {
    for (const auto & edge : msg->edges) {
      const auto from_it = node_positions.find(edge.from_id);
      const auto to_it = node_positions.find(edge.to_id);
      if (from_it == node_positions.end() || to_it == node_positions.end()) {
        continue;
      }
      addEdgeVisual(from_it->second, to_it->second, edge);
    }
  }
}

void GraphDisplay::updateStyles()
{
  rebuildVisuals();
  if (context_ != nullptr) {
    context_->queueRender();
  }
}

}  // namespace autonomy_msgs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autonomy_msgs::GraphDisplay, rviz_common::Display)
