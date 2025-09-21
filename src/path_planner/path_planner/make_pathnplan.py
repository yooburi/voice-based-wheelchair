#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

import tf2_ros
import yaml


def yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)


class StraightPathPlanner(Node):
    def __init__(self):
        super().__init__('straight_path_planner')

        # Parameters
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
        self.step_dist = float(self.declare_parameter('step_dist', 0.1) \
            .get_parameter_value().double_value)

        # TF buffer/listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers / Subscribers
        self.plan_pub = self.create_publisher(Path, self.plan_topic, 10)
        self.target_sub = self.create_subscription(String, self.target_topic, self._on_target, 10)

        # Load locations
        self.locations: Dict[str, Tuple[float, float, Optional[float]]] = {}
        self._load_locations()

        self.get_logger().info(
            f"StraightPathPlanner up: listen {self.target_topic}, publish {self.plan_topic}, yaml={self.location_yaml}"
        )

    def _load_locations(self):
        try:
            with open(self.location_yaml, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().warn(f"Failed to read YAML '{self.location_yaml}': {e}")
            self.locations = {}
            return

        # Accept either top-level 'labels' mapping or direct mapping
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
        self.get_logger().info(f"Loaded {len(self.locations)} labels from YAML")

    def _lookup_current_xy(self) -> Optional[Tuple[float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.5)
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            return (x, y)
        except Exception as e:
            self.get_logger().warn(f"TF {self.frame_id}->{self.base_frame} unavailable: {e}")
            return None

    def _build_path(self, start_xy: Tuple[float, float], goal_xy: Tuple[float, float], goal_yaw: Optional[float]) -> Path:
        sx, sy = start_xy
        gx, gy = goal_xy
        dx = gx - sx
        dy = gy - sy
        dist = math.hypot(dx, dy)

        # At least two points (start and goal)
        n = max(2, int(dist / max(self.step_dist, 1e-3)) + 1)
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        yaw_along = math.atan2(dy, dx) if dist > 1e-6 else (goal_yaw if goal_yaw is not None else 0.0)

        for i in range(n):
            t = i / (n - 1)
            x = sx + dx * t
            y = sy + dy * t
            # Orientation: along the path; for the last point, prefer goal_yaw if provided
            if i == n - 1 and goal_yaw is not None:
                yaw = goal_yaw
            else:
                yaw = yaw_along

            _, _, qz, qw = yaw_to_quaternion(yaw)
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)

        return path

    def _on_target(self, msg: String):
        label = (msg.data or '').strip()
        if not label:
            return

        # Reload YAML each time in case it changed on disk
        self._load_locations()

        if label not in self.locations:
            self.get_logger().warn(
                f"Unknown label '{label}'. Known: {list(self.locations.keys())}"
            )
            return

        goal_x, goal_y, goal_yaw = self.locations[label]

        cur = self._lookup_current_xy()
        if cur is None:
            self.get_logger().warn("Cannot build path without current pose.")
            return

        path = self._build_path(cur, (goal_x, goal_y), goal_yaw)
        self.plan_pub.publish(path)
        self.get_logger().info(
            f"Published straight Path to '{label}' -> ({goal_x:.2f}, {goal_y:.2f}) with {len(path.poses)} poses"
        )


def main(args=None):
    rclpy.init(args=args)
    node = StraightPathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
