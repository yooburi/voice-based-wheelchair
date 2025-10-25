#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rosidl_runtime_py.utilities import get_service

import yaml
try:
    from ament_index_python.packages import get_package_share_directory  # type: ignore
except Exception:
    get_package_share_directory = None  # type: ignore


def yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)


class Nav2PathPlannerBridge(Node):
    """
    Bridge node: String label on /target_location -> Nav2 ComputePathToPose service call.
    Republish the returned nav_msgs/Path to /plan for compatibility with existing path_follower.
    """

    def __init__(self):
        super().__init__('nav2_path_planner_bridge')

        # Parameters
        self.target_topic = self.declare_parameter('target_topic', '/target_location') \
            .get_parameter_value().string_value
        self.plan_topic = self.declare_parameter('plan_topic', '/plan') \
            .get_parameter_value().string_value
        self.frame_id = self.declare_parameter('frame_id', 'map') \
            .get_parameter_value().string_value
        self.location_yaml = self.declare_parameter('location_yaml', 'config/location/location.yaml') \
            .get_parameter_value().string_value
        self.planner_service_name = self.declare_parameter(
            'planner_service', '/planner_server/compute_path_to_pose'
        ).get_parameter_value().string_value
        self.service_timeout = float(self.declare_parameter('service_timeout', 5.0) \
            .get_parameter_value().double_value)

        # Resolve service type dynamically to avoid import errors on some installations
        self.srv_type = None
        try:
            self.srv_type = get_service('nav2_msgs/srv/ComputePathToPose')
        except Exception as e:
            self.get_logger().error(
                "Service type nav2_msgs/srv/ComputePathToPose not found. "
                "Ensure Nav2 is properly installed (ros-humble-nav2-msgs/nav2-planner) "
                "and sourced. Error: %s" % (e,)
            )

        # Client / Pub / Sub
        if self.srv_type is not None:
            self.client = self.create_client(self.srv_type, self.planner_service_name)
        else:
            self.client = None
        self.plan_pub = self.create_publisher(Path, self.plan_topic, 10)
        self.target_sub = self.create_subscription(String, self.target_topic, self._on_target, 10)

        # Locations cache
        self.locations: Dict[str, Tuple[float, float, Optional[float]]] = {}
        self._load_locations()

        self._busy = False

        self.get_logger().info(
            f"Nav2PathPlannerBridge up: in={self.target_topic}, service={self.planner_service_name}, out={self.plan_topic}"
        )

    def _resolve_yaml_path(self, yaml_path: str) -> str:
        # If not absolute or not exists, try package share/config/location/location.yaml then a known workspace path
        if os.path.isabs(yaml_path) and os.path.exists(yaml_path):
            return yaml_path
        candidate = None
        if get_package_share_directory is not None:
            try:
                share_dir = get_package_share_directory('path_planner')
                candidate = os.path.join(share_dir, 'config', 'location', 'location.yaml')
            except Exception:
                candidate = None
        if (candidate is None) or (not os.path.exists(candidate)):
            candidate2 = '/home/yoo/workspace/dolchair_ws/config/location/location.yaml'
            if os.path.exists(candidate2):
                candidate = candidate2
        return candidate or yaml_path

    def _load_locations(self):
        yaml_path = self._resolve_yaml_path(self.location_yaml)
        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            if yaml_path != self.location_yaml:
                self.get_logger().info(f"location_yaml resolved to: {yaml_path}")
        except Exception as e:
            self.get_logger().warn(f"Failed to read YAML '{yaml_path}': {e}")
            self.locations = {}
            return

        labels = data.get('labels', data if isinstance(data, dict) else {})
        locs: Dict[str, Tuple[float, float, Optional[float]]] = {}
        if isinstance(labels, dict):
            for name, spec in labels.items():
                try:
                    x = float(spec.get('x'))
                    y = float(spec.get('y'))
                    yaw = spec.get('yaw')
                    yaw = float(yaw) if yaw is not None else None
                    locs[str(name)] = (x, y, yaw)
                except Exception:
                    self.get_logger().warn(f"Skip invalid label entry: {name} -> {spec}")
        self.locations = locs
        self.get_logger().info(f"Loaded {len(self.locations)} labels: {list(self.locations.keys())}")

    def _on_target(self, msg: String):
        if self._busy:
            self.get_logger().warn('Planner busy; ignoring incoming target')
            return

        label = (msg.data or '').strip()
        if not label:
            return

        # Reload on each request to pick up edits
        self._load_locations()

        if label not in self.locations:
            self.get_logger().warn(f"Unknown label '{label}'. Known: {list(self.locations.keys())}")
            return

        gx, gy, gyaw = self.locations[label]

        # Build goal
        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.position.z = 0.0
        if gyaw is None:
            goal.pose.orientation.w = 1.0
        else:
            _, _, qz, qw = yaw_to_quaternion(gyaw)
            goal.pose.orientation.z = qz
            goal.pose.orientation.w = qw

        if self.client is None or self.srv_type is None:
            self.get_logger().error('Planner service type unavailable; cannot call planner.')
            return

        # Prepare service request
        req = self.srv_type.Request()
        req.goal = goal
        req.use_start = False  # Use AMCL current pose as start

        # Call service
        if not self.client.wait_for_service(timeout_sec=self.service_timeout):
            self.get_logger().error(
                f"Planner service '{self.planner_service_name}' unavailable"
            )
            return

        self.get_logger().info(f"Requesting path to '{label}' -> ({gx:.2f}, {gy:.2f})")
        self._busy = True
        future = self.client.call_async(req)

        def _done_cb(fut):
            self._busy = False
            try:
                resp = fut.result()
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
                return

            if resp is None or resp.path is None or len(resp.path.poses) == 0:
                self.get_logger().warn("Empty path returned from Nav2 planner")
                return

            # Republish the path for compatibility
            path: Path = resp.path
            self.plan_pub.publish(path)
            self.get_logger().info(f"Published Nav2 path with {len(path.poses)} poses")

        future.add_done_callback(_done_cb)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2PathPlannerBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
