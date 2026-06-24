"""Collection coordinator ROS node."""

from __future__ import annotations

import json
import time

import rclpy
from autonomy_msgs.msg import Graph
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from autonomy_task.coordinator import Coordinator
from autonomy_task.waypoint import Status, Waypoint

_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

_COLORS = {
    Status.PENDING: (0.9, 0.9, 0.2, 0.9),
    Status.IN_PROGRESS: (0.2, 0.6, 1.0, 1.0),
    Status.COLLECTED: (0.15, 0.9, 0.25, 1.0),
    Status.FAILED: (0.95, 0.2, 0.2, 1.0),
    Status.SKIPPED: (0.55, 0.55, 0.55, 0.7),
    Status.BLOCKED: (0.7, 0.3, 0.9, 0.9),
}


def _markers(frame: str, stamp, waypoints: list[Waypoint]) -> MarkerArray:
    out = MarkerArray()
    out.markers.append(Marker(action=Marker.DELETEALL))
    for i, wp in enumerate(waypoints):
        rgba = _COLORS.get(wp.status, (1.0, 1.0, 1.0, 1.0))
        s = 0.14 if wp.status == Status.COLLECTED else 0.18
        sphere = Marker()
        sphere.header.frame_id = frame
        sphere.header.stamp = stamp
        sphere.ns = 'wps'
        sphere.id = i
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = wp.x
        sphere.pose.position.y = wp.y
        sphere.pose.position.z = wp.z
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = s
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = map(float, rgba)
        out.markers.append(sphere)

        label = Marker()
        label.header = sphere.header
        label.ns = 'labels'
        label.id = i
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = wp.x
        label.pose.position.y = wp.y
        label.pose.position.z = wp.z + 0.25
        label.pose.orientation.w = 1.0
        label.scale.z = 0.18
        label.color.r = label.color.g = label.color.b = 1.0
        label.color.a = 0.95
        label.text = f'{wp.id}\n{wp.skip_reason}' if wp.skip_reason else wp.id
        out.markers.append(label)
    return out


class CoordinatorNode(Node):
    def __init__(self) -> None:
        super().__init__('collection_coordinator')
        self.declare_parameter('config_file', '')
        self.declare_parameter('num_robots', 0)
        self.declare_parameter('graph_topic', '')
        self.declare_parameter('dataset_root', '')
        self.declare_parameter('dataset_repo_id', '')
        cfg_path = self.get_parameter('config_file').get_parameter_value().string_value
        if not cfg_path:
            raise RuntimeError('config_file is required')

        num_robots = int(self.get_parameter('num_robots').value)
        graph_topic = self.get_parameter('graph_topic').get_parameter_value().string_value
        self._coord = Coordinator.from_config(
            self,
            cfg_path,
            on_update=self._publish,
            num_robots=num_robots,
            graph_topic=graph_topic,
        )
        self._cfg = self._coord.config
        self._cfg.apply_dataset(
            dataset_root=self.get_parameter('dataset_root').get_parameter_value().string_value,
            dataset_repo_id=self.get_parameter('dataset_repo_id').get_parameter_value().string_value,
        )
        robots = self._cfg.robot_names()
        self._cfg.ensure_output_dirs(robots)
        self.get_logger().info(f'state file: {self._cfg.state_file}')
        for name in robots:
            self.get_logger().info(
                f'{name} dataset: {self._cfg.dataset_root_for(name)} '
                f'({self._cfg.dataset_repo_id})')

        self._collection_ready = False
        self._startup_t0 = time.time()
        self._graph_wait_logged = False
        self._costmap_wait_logged = False
        self._servers_wait_logged = False
        self._servers_wait_last_log = 0.0

        self._status_pub = self.create_publisher(String, '~/collection_status', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '~/waypoint_markers', 10)

        # Register recording services before any blocking work so they are
        # discoverable as soon as the executor spins.
        if self._cfg.record.enabled:
            recording_cb = ReentrantCallbackGroup()
            self.create_service(
                Trigger, '~/start_recording', self._on_start_recording,
                callback_group=recording_cb)
            self.create_service(
                Trigger, '~/stop_recording', self._on_stop_recording,
                callback_group=recording_cb)
            self.get_logger().info(
                'recording services: /collection_coordinator/start_recording, '
                '/collection_coordinator/stop_recording')

        if self._cfg.waypoint_source == 'graph':
            if self._cfg.graph.per_robot:
                for name in robots:
                    topic = self._cfg.graph_topic_for(name)
                    self.get_logger().info(f'graph topic [{name}]: {topic}')
                    self.create_subscription(
                        Graph,
                        topic,
                        self._make_graph_cb(name),
                        _LATCHED_QOS,
                    )
            else:
                topic = self._cfg.graph.topic
                self.get_logger().info(f'graph topic: {topic}')
                self.create_subscription(
                    Graph, topic, self._coord.set_graph, _LATCHED_QOS)
        elif self._cfg.filter.require_navigable and not self._cfg.uses_per_robot_maps:
            topic = self._cfg.filter.map_topic
            if not topic.startswith('/'):
                topic = f'/{robots[0]}/{topic}'
            self.create_subscription(OccupancyGrid, topic, self._coord.set_grid, _LATCHED_QOS)

        if self._needs_map() and self._cfg.uses_per_robot_maps:
            for name in robots:
                topic = self._cfg.costmap_topic_for(name)
                label = 'map' if not self._cfg.filter.use_global_costmap else 'costmap'
                self.get_logger().info(f'{label} topic [{name}]: {topic}')
                self.create_subscription(
                    OccupancyGrid,
                    topic,
                    self._make_costmap_cb(name),
                    _LATCHED_QOS,
                )

        self.create_timer(0.5, self._on_startup_tick)
        self.create_timer(1.0 / self._cfg.tick_hz, self._on_collection_tick)
        self.create_timer(1.0 / self._cfg.status_hz, self._publish)

    def _needs_map(self) -> bool:
        filt = self._cfg.filter
        return filt.require_navigable or filt.enforce_map_bounds

    def _make_graph_cb(self, robot: str):
        def _cb(msg: Graph) -> None:
            self._coord.set_robot_graph(robot, msg)
        return _cb

    def _make_costmap_cb(self, robot: str):
        def _cb(msg: OccupancyGrid) -> None:
            self._coord.set_robot_grid(robot, msg)
        return _cb

    def _on_startup_tick(self) -> None:
        if self._collection_ready:
            return

        now = time.time()
        elapsed = now - self._startup_t0

        if self._cfg.waypoint_source == 'graph' and not self._coord.graph_ready:
            pending = self._coord.pending_graph_robots()
            if not self._graph_wait_logged:
                topics = self._coord.graph_topics()
                self.get_logger().info(
                    f'waiting for graph on {", ".join(topics)} ...')
                self._graph_wait_logged = True
            if elapsed > self._cfg.graph.wait_sec:
                detail = ', '.join(pending) if pending else 'unknown'
                self.get_logger().error(
                    f'graph not ready within {self._cfg.graph.wait_sec}s '
                    f'(pending: {detail}); collection disabled')
            return

        pending_costmap = self._coord.pending_costmap_robots()
        if pending_costmap:
            if not self._costmap_wait_logged:
                topics = self._coord.map_topics()
                self.get_logger().info(
                    f'waiting for map: {", ".join(pending_costmap)} '
                    f'({", ".join(topics)}) ...')
                self._costmap_wait_logged = True
            if elapsed > self._cfg.nav.server_wait_sec:
                self.get_logger().error(
                    f'map not received for {", ".join(pending_costmap)} within '
                    f'{self._cfg.nav.server_wait_sec}s; collection disabled')
            return

        pending: list[str] = []
        for robot in self._coord.robots:
            missing = robot.pending_servers()
            if missing:
                pending.append(f'{robot.namespace}({",".join(missing)})')
        if pending:
            if not self._servers_wait_logged:
                self.get_logger().info(
                    f'waiting for robot services: {", ".join(pending)} ...')
                self._servers_wait_logged = True
                self._servers_wait_last_log = now
            elif now - self._servers_wait_last_log >= 5.0:
                self.get_logger().info(
                    f'still waiting for robot services: {", ".join(pending)} '
                    f'({elapsed:.0f}s)')
                self._servers_wait_last_log = now
            if elapsed > self._cfg.nav.server_wait_sec:
                self.get_logger().error(
                    f'robot servers not ready within {self._cfg.nav.server_wait_sec}s: '
                    f'{", ".join(pending)}; collection disabled')
            return

        self._collection_ready = True
        self.get_logger().info(
            f'ready: {len(self._cfg.robot_names())} robots, '
            f'{len(self._coord.waypoints)} wps')
        self._publish()

    def _on_collection_tick(self) -> None:
        if not self._collection_ready:
            return
        self._coord.tick()

    def _on_start_recording(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if not self._cfg.record.enabled:
            response.success = False
            response.message = 'recording disabled in config'
            return response
        ok, message = self._coord.start_recording()
        response.success = ok
        response.message = message
        return response

    def _on_stop_recording(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if not self._cfg.record.enabled:
            response.success = False
            response.message = 'recording disabled in config'
            return response
        ok, message = self._coord.stop_recording()
        response.success = ok
        response.message = message
        return response

    def _publish(self) -> None:
        if not hasattr(self, '_status_pub'):
            return
        payload = {
            'summary': self._coord.summary(),
            'finished': self._coord.finished,
            'collection_ready': self._collection_ready,
            'output': {
                'state_file': self._cfg.state_file,
                'dataset_repo_id': self._cfg.dataset_repo_id,
                'datasets': {
                    r.namespace: self._cfg.dataset_root_for(r.namespace)
                    for r in self._coord.robots
                },
            },
            'robots': [
                {
                    'name': r.namespace,
                    'phase': r.phase.value,
                    'active': r.active.id if r.active else '',
                    'pose': (
                        {'x': r.pose().x, 'y': r.pose().y, 'yaw': r.pose().yaw}
                        if r.pose() else None
                    ),
                }
                for r in self._coord.robots
            ],
            'trajectories': self._coord.trajectory_summary(),
            'waypoints': [wp.as_dict() for wp in self._coord.waypoints],
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._status_pub.publish(msg)
        stamp = self.get_clock().now().to_msg()
        self._marker_pub.publish(
            _markers(self._cfg.marker_frame, stamp, self._coord.waypoints))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoordinatorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
