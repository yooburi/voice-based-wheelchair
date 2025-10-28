#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

import tf2_ros
import yaml

try:
    from ament_index_python.packages import get_package_share_directory  # type: ignore
except Exception:
    get_package_share_directory = None  # type: ignore

try:
    from nav2_msgs.action import ComputePathToPose  # type: ignore
except Exception:
    ComputePathToPose = None  # type: ignore


def yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)


class Nav2PathPlanner(Node):
    """
    Subscribe: target location label (String)
    Action: call Nav2 ComputePathToPose to generate a plan
    Publish: nav_msgs/Path on /plan (compatible with existing path_follower)
    """

    def __init__(self) -> None:
        super().__init__('nav2_path_planner')

        # Parameters (aligned with make_pathnplan for drop-in replacement)
        self.target_topic = self.declare_parameter('target_topic', '/target_location') \
            .get_parameter_value().string_value
        self.plan_topic = self.declare_parameter('plan_topic', '/plan') \
            .get_parameter_value().string_value
        self.frame_id = self.declare_parameter('frame_id', 'map') \
            .get_parameter_value().string_value
        self.base_frame = self.declare_parameter('base_frame', 'base_link') \
            .get_parameter_value().string_value
        self.location_yaml = self.declare_parameter('location_yaml', 'config/location/location.yaml') \
            .get_parameter_value().string_value

        # TF (used only for diagnostics or potential future use)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Nav2 action client(s)
        self.compute_client = None
        self.compute_client_alt = None
        if ComputePathToPose is None:
            self.get_logger().error('nav2_msgs not found. Please ensure nav2_msgs is installed and declared as dependency.')
        else:
            # Nav2 planner action name (Humble): '/compute_path_to_pose'
            self.compute_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
            # Fallback for deployments that prefix with node name
            self.compute_client_alt = ActionClient(self, ComputePathToPose, '/planner_server/compute_path_to_pose')

        # Pub/Sub
        self.plan_pub = self.create_publisher(Path, self.plan_topic, 10)
        self.target_sub = self.create_subscription(String, self.target_topic, self._on_target, 10)

        # Locations
        self.locations: Dict[str, Tuple[float, float, Optional[float]]] = {}
        self._load_locations()

        self.get_logger().info(
            f"Nav2PathPlanner up: listen {self.target_topic}, publish {self.plan_topic}, yaml={self.location_yaml}"
        )

    def _load_locations(self) -> None:
        yaml_path = self.location_yaml
        # Resolve to an existing file if a relative/bad path is given
        if not os.path.isabs(yaml_path) or not os.path.exists(yaml_path):
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
            if candidate and os.path.exists(candidate):
                yaml_path = candidate

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

    def _build_goal_pose(self, x: float, y: float, yaw: Optional[float]) -> PoseStamped:
        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.0
        if yaw is None:
            goal.pose.orientation.w = 1.0
        else:
            _, _, qz, qw = yaw_to_quaternion(yaw)
            goal.pose.orientation.z = qz
            goal.pose.orientation.w = qw
        return goal

    def _on_target(self, msg: String) -> None:
        label = (msg.data or '').strip()
        if not label:
            return

        # Reload YAML each time to pick up runtime edits
        self._load_locations()

        if label not in self.locations:
            self.get_logger().warn(
                f"Unknown label '{label}'. Known: {list(self.locations.keys())}"
            )
            return

        gx, gy, gyaw = self.locations[label]
        goal = self._build_goal_pose(gx, gy, gyaw)

        # Ensure action server is up (brief wait to avoid blocking)
        client = self.compute_client
        if (client is None) or (not client.wait_for_server(timeout_sec=0.5)):
            # Try fallback name
            client = self.compute_client_alt
            if (client is None) or (not client.wait_for_server(timeout_sec=0.5)):
                self.get_logger().warn('Nav2 planner action not available: /compute_path_to_pose or /planner_server/compute_path_to_pose')
                return

        # ComputePathToPose type guaranteed by compute_client check above

        goal_msg = ComputePathToPose.Goal()
        # When use_start=False, Nav2 uses the robot's current pose internally
        goal_msg.use_start = False
        # Fill start minimally for completeness
        goal_msg.start = PoseStamped()
        goal_msg.start.header.frame_id = self.frame_id
        goal_msg.goal = goal

        self.get_logger().info(f"Requesting Nav2 path to '{label}' -> ({gx:.2f}, {gy:.2f})")
        send_future = client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_compute_feedback,
        )
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Nav2 action send failed: {e}")
            return

        if not goal_handle.accepted:
            self.get_logger().error('Nav2 planner rejected ComputePathToPose goal')
            return

        self.get_logger().info('Nav2 planner accepted goal, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_compute_feedback(self, feedback_msg) -> None:
        # Optionally handle feedback (suppress verbose logs by default)
        # feedback = feedback_msg.feedback
        pass

    def _on_result(self, future) -> None:
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().error(f"Nav2 ComputePathToPose result failed: {e}")
            return

        if result is None:
            self.get_logger().error('Nav2 result is None')
            return

        path: Path = result.path
        if not path.poses:
            self.get_logger().warn('Nav2 returned empty path')
            return

        # Normalize header if missing
        if not path.header.frame_id:
            path.header.frame_id = self.frame_id
        if path.header.stamp.sec == 0 and path.header.stamp.nanosec == 0:
            path.header.stamp = self.get_clock().now().to_msg()

        self.plan_pub.publish(path)
        self.get_logger().info(f"Published Nav2 Path with {len(path.poses)} poses (frame={path.header.frame_id})")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()