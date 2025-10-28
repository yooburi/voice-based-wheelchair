#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

import yaml
import tf2_ros

try:
    from ament_index_python.packages import get_package_share_directory  # type: ignore
except Exception:
    get_package_share_directory = None  # type: ignore

try:
    from nav2_msgs.action import NavigateToPose  # type: ignore
except Exception:
    NavigateToPose = None  # type: ignore


def yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)


class Nav2NavigateToPose(Node):
    """
    Listen to /target_location and send Nav2 NavigateToPose goal via BT Navigator.
    This is the Full Nav2 path: Compute -> Follow (RPP) -> Recoveries via BT.
    """

    def __init__(self) -> None:
        super().__init__('nav2_navigate_to_pose')

        # Parameters
        self.target_topic = self.declare_parameter('target_topic', '/target_location') \
            .get_parameter_value().string_value
        self.frame_id = self.declare_parameter('frame_id', 'map') \
            .get_parameter_value().string_value
        self.location_yaml = self.declare_parameter('location_yaml', 'config/location/location.yaml') \
            .get_parameter_value().string_value

        # TF (for potential diagnostics)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Action client(s)
        self.nav_client = None
        self.nav_client_alt = None
        if NavigateToPose is None:
            self.get_logger().error('nav2_msgs not found; ensure nav2_msgs is installed and declared as dependency.')
        else:
            self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
            self.nav_client_alt = ActionClient(self, NavigateToPose, '/bt_navigator/navigate_to_pose')

        # Pub/Sub
        self.sub = self.create_subscription(String, self.target_topic, self._on_target, 10)

        # Locations
        self.locations: Dict[str, Tuple[float, float, Optional[float]]] = {}
        self._load_locations()

        self.get_logger().info(
            f"Nav2NavigateToPose: listening {self.target_topic}, yaml={self.location_yaml}"
        )

    def _load_locations(self) -> None:
        yaml_path = self.location_yaml
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

        self._load_locations()
        if label not in self.locations:
            self.get_logger().warn(
                f"Unknown label '{label}'. Known: {list(self.locations.keys())}"
            )
            return

        gx, gy, gyaw = self.locations[label]
        pose = self._build_goal_pose(gx, gy, gyaw)

        client = self.nav_client
        if (client is None) or (not client.wait_for_server(timeout_sec=0.5)):
            client = self.nav_client_alt
            if (client is None) or (not client.wait_for_server(timeout_sec=0.5)):
                self.get_logger().warn('BT Navigator action not available: /navigate_to_pose or /bt_navigator/navigate_to_pose')
                return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f"Sending NavigateToPose to '{label}' -> ({gx:.2f}, {gy:.2f})")
        send_future = client.send_goal_async(goal_msg, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Nav2 action send failed: {e}")
            return

        if not goal_handle.accepted:
            self.get_logger().error('NavigateToPose goal rejected')
            return
        self.get_logger().info('NavigateToPose goal accepted; waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg) -> None:
        # Feedback can be logged if needed (suppress verbose output by default)
        # feedback = feedback_msg.feedback
        pass

    def _on_result(self, future) -> None:
        try:
            result = future.result().result
            status = future.result().status
        except Exception as e:
            self.get_logger().error(f"NavigateToPose failed: {e}")
            return

        self.get_logger().info(f"NavigateToPose finished with status={status}; result={result}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2NavigateToPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

