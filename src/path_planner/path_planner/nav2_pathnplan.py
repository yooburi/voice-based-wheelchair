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
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

import tf2_ros
import yaml

try:
    from ament_index_python.packages import get_package_share_directory  # type: ignore
except Exception:
    get_package_share_directory = None  # type: ignore


def yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)


class Nav2PathPlanner(Node):
    """
    Subscribe: target location label (std_msgs/String)
    Action: send Nav2 NavigateToPose goal
    """

    def __init__(self) -> None:
        super().__init__('nav2_path_planner')

        # Parameters
        self.target_topic = self.declare_parameter('target_topic', '/target_location') \
            .get_parameter_value().string_value
        self.frame_id = self.declare_parameter('frame_id', 'map') \
            .get_parameter_value().string_value
        self.base_frame = self.declare_parameter('base_frame', 'base_link') \
            .get_parameter_value().string_value
        self.location_yaml = self.declare_parameter('location_yaml', 'config/location/location.yaml') \
            .get_parameter_value().string_value

        # TF (for diagnostics / future use)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Target subscriber
        self.target_sub = self.create_subscription(String, self.target_topic, self._on_target, 10)

        # Load locations
        self.locations: Dict[str, Tuple[float, float, Optional[float]]] = {}
        self._load_locations()

        self.get_logger().info(
            f"Nav2PathPlanner up. listen={self.target_topic}, yaml={self.location_yaml}"
        )

    def _load_locations(self) -> None:
        yaml_path = self.location_yaml

        # Resolve fallback path
        if (not os.path.isabs(yaml_path)) or (not os.path.exists(yaml_path)):
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
            self.get_logger().warning(f"Failed to read YAML '{yaml_path}': {e}")
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
                    self.get_logger().warning(f"Skip invalid label entry: {name} -> {spec}")
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

        # Reload YAML at runtime
        self._load_locations()

        if label not in self.locations:
            self.get_logger().warning(
                f"Unknown label '{label}'. Known: {list(self.locations.keys())}"
            )
            return

        gx, gy, gyaw = self.locations[label]
        goal = self._build_goal_pose(gx, gy, gyaw)

        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning('Nav2 action server /navigate_to_pose not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal
        # goal_msg.behavior_tree = "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"

        self.get_logger().info(f"Request Nav2 navigation to '{label}' -> ({gx:.2f}, {gy:.2f})")

        send_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_nav_feedback,
        )
        send_future.add_done_callback(self._on_goal_response)

    def _on_nav_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        # 일부 버전에선 distance_remaining이 없을 수도 있으니 getattr로 안전 처리
        dist = getattr(fb, 'distance_remaining', None)
        if dist is not None:
            self.get_logger().info(f"Navigating... distance_remaining: {dist:.2f} m")

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Nav2 action send failed: {e}")
            return

        if not goal_handle.accepted:
            self.get_logger().error('Nav2 navigate_to_pose goal was rejected')
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        try:
            res = future.result()
            status = res.status
            result = res.result
        except Exception as e:
            self.get_logger().error(f"Nav2 navigation result failed: {e}")
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Navigation Succeeded! (Result: {result})")
        else:
            self.get_logger().warning(f"Navigation Failed with status: {status}")


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
