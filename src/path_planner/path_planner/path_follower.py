#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

import tf2_ros

from std_msgs.msg import Int8
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PathFollower(Node):
    def __init__(self) -> None:
        super().__init__('path_follower')

        # Parameters
        self.plan_topic = self.declare_parameter('plan_topic', '/plan') \
            .get_parameter_value().string_value
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel') \
            .get_parameter_value().string_value
        self.turn_topic = self.declare_parameter('turn_topic', '/turn_dir') \
            .get_parameter_value().string_value
        self.global_frame = self.declare_parameter('frame_id', 'map') \
            .get_parameter_value().string_value
        self.base_frame = self.declare_parameter('base_frame', 'base_link') \
            .get_parameter_value().string_value

        self.yaw_tol = float(self.declare_parameter('yaw_tol', 0.1) \
            .get_parameter_value().double_value)  # rad (legacy; unused if hysteresis used)
        # Hysteresis thresholds for heading alignment (radians)
        self.yaw_align_enter = float(self.declare_parameter('yaw_align_enter', 0.15) \
            .get_parameter_value().double_value)  # Enter DRIVE when below this
        self.yaw_align_exit = float(self.declare_parameter('yaw_align_exit', 0.25) \
            .get_parameter_value().double_value)   # Leave DRIVE when above this
        self.goal_tol = float(self.declare_parameter('goal_tol', 0.2) \
            .get_parameter_value().double_value)  # meters
        self.waypoint_lookahead = float(self.declare_parameter('waypoint_lookahead', 0.3) \
            .get_parameter_value().double_value)  # meters to advance waypoint

        self.angular_speed = float(self.declare_parameter('angular_speed', 0.5) \
            .get_parameter_value().double_value)  # rad/s
        self.linear_speed = float(self.declare_parameter('linear_speed', 150.0) \
            .get_parameter_value().double_value)  # m/s
        self.rate_hz = float(self.declare_parameter('rate_hz', 20.0) \
            .get_parameter_value().double_value)

        # Visualization
        self.viz_enable = bool(self.declare_parameter('viz_enable', True) \
            .get_parameter_value().bool_value)
        self.marker_topic = self.declare_parameter('marker_topic', '/path_follower_markers') \
            .get_parameter_value().string_value
        self.arrow_len = float(self.declare_parameter('arrow_length', 0.4) \
            .get_parameter_value().double_value)
        self.arrow_w = float(self.declare_parameter('arrow_width', 0.05) \
            .get_parameter_value().double_value)
        self.arrow_h = float(self.declare_parameter('arrow_height', 0.05) \
            .get_parameter_value().double_value)
        self.wp_radius = float(self.declare_parameter('waypoint_radius', 0.1) \
            .get_parameter_value().double_value)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Pub/Sub
        self.turn_pub = self.create_publisher(Int8, self.turn_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.plan_sub = self.create_subscription(Path, self.plan_topic, self._on_plan, 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)

        # State
        self._plan: List[Tuple[float, float]] = []
        self._plan_stamp = Time()
        self._idx = 0
        self._mode = 'ROTATE'  # ROTATE or DRIVE (hysteresis)

        # Control timer
        self.timer = self.create_timer(1.0 / max(self.rate_hz, 1.0), self._on_timer)

        self.get_logger().info(
            f"PathFollower up: plan={self.plan_topic}, cmd={self.cmd_vel_topic}, turn={self.turn_topic}"
        )

    def _on_plan(self, msg: Path) -> None:
        # Store plan as (x, y) list; keep the frame id if provided
        self.global_frame = msg.header.frame_id or self.global_frame
        self._plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._plan_stamp = self.get_clock().now()
        self._idx = 0
        self.get_logger().info(f"Received plan with {len(self._plan)} poses in frame '{self.global_frame}'")

    def _lookup_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.3)
            )
        except Exception as e:
            self.get_logger().warn(f"TF {self.global_frame}->{self.base_frame} unavailable: {e}")
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y

        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w

        # yaw from quaternion
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return (x, y, yaw)

    def _publish_turn(self, direction: int) -> None:
        msg = Int8()
        msg.data = int(max(-1, min(1, direction)))
        self.turn_pub.publish(msg)

    def _stop(self) -> None:
        twist = Twist()
        self.cmd_pub.publish(twist)
        self._publish_turn(0)

    def _on_timer(self) -> None:
        # No plan
        if not self._plan:
            self._stop()
            return

        pose = self._lookup_pose()
        if pose is None:
            self._stop()
            return

        x, y, yaw = pose

        # Advance waypoint if close
        while self._idx < len(self._plan):
            wx, wy = self._plan[self._idx]
            if math.hypot(wx - x, wy - y) <= self.waypoint_lookahead and self._idx < len(self._plan) - 1:
                self._idx += 1
            else:
                break

        wx, wy = self._plan[self._idx]
        dx = wx - x
        dy = wy - y
        dist = math.hypot(dx, dy)

        # Goal reached?
        if self._idx >= len(self._plan) - 1 and dist <= self.goal_tol:
            self.get_logger().info_once("Goal reached. Stopping.")
            self._stop()
            # Publish final markers as stopped state
            if self.viz_enable:
                self._publish_markers(x, y, yaw, (wx, wy))
            return

        # Heading control with hysteresis
        target_yaw = math.atan2(dy, dx) if dist > 1e-6 else yaw
        yaw_err = normalize_angle(target_yaw - yaw)
        abs_err = abs(yaw_err)

        # State transitions (hysteresis)
        if self._mode == 'DRIVE':
            if abs_err >= self.yaw_align_exit:
                self._mode = 'ROTATE'
        else:  # ROTATE
            if abs_err <= self.yaw_align_enter:
                self._mode = 'DRIVE'

        twist = Twist()
        if self._mode == 'ROTATE':
            # rotate in place
            twist.angular.z = self.angular_speed * (1.0 if yaw_err > 0.0 else -1.0)
            twist.linear.x = 0.0
            # Bridge convention: left=-1, right=+1
            self._publish_turn(-1 if yaw_err > 0 else +1)
        else:
            # DRIVE: go straight and ignore small errors; keep turn_dir=0
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            self._publish_turn(0)

        self.cmd_pub.publish(twist)

        # Visualization markers
        if self.viz_enable:
            self._publish_markers(x, y, yaw, (wx, wy))

    def _publish_markers(self, x: float, y: float, yaw: float, waypoint: Tuple[float, float]) -> None:
        now = self.get_clock().now().to_msg()
        # Heading arrow (id 0)
        arrow = Marker()
        arrow.header.frame_id = self.global_frame
        arrow.header.stamp = now
        arrow.ns = 'path_follower'
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = x
        arrow.pose.position.y = y
        arrow.pose.position.z = 0.05
        # orientation from yaw
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        arrow.pose.orientation.z = qz
        arrow.pose.orientation.w = qw
        arrow.scale.x = self.arrow_len
        arrow.scale.y = self.arrow_w
        arrow.scale.z = self.arrow_h
        arrow.color.a = 0.8
        arrow.color.r = 0.2
        arrow.color.g = 0.5
        arrow.color.b = 1.0
        arrow.lifetime = Duration(seconds=0.5).to_msg()
        self.marker_pub.publish(arrow)

        # Current waypoint sphere (id 1)
        wx, wy = waypoint
        wp = Marker()
        wp.header.frame_id = self.global_frame
        wp.header.stamp = now
        wp.ns = 'path_follower'
        wp.id = 1
        wp.type = Marker.SPHERE
        wp.action = Marker.ADD
        wp.pose.position.x = wx
        wp.pose.position.y = wy
        wp.pose.position.z = 0.05
        wp.pose.orientation.w = 1.0
        wp.scale.x = self.wp_radius * 2.0
        wp.scale.y = self.wp_radius * 2.0
        wp.scale.z = self.wp_radius * 2.0
        wp.color.a = 0.8
        wp.color.r = 0.2
        wp.color.g = 0.5
        wp.color.b = 1.0
        wp.lifetime = Duration(seconds=0.5).to_msg()
        self.marker_pub.publish(wp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
