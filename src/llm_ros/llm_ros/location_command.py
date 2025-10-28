#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
import tf2_ros
import yaml
import os
import math
from typing import Dict, Tuple, Optional

# Helper function to convert yaw to quaternion (copied from make_pathnplan.py)
def yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)

class LocationCommandNode(Node):
    def __init__(self):
        super().__init__('location_command_node')

        # Parameters
        self.in_topic = self.declare_parameter('in_topic', '/text_to_location') \
            .get_parameter_value().string_value
        self.target_location_topic = self.declare_parameter('target_location_topic', '/target_location') \
            .get_parameter_value().string_value
        self.location_yaml_path = self.declare_parameter('location_yaml_path', '/home/yoo/workspace/dolchair_ws/config/location/location.yaml') \
            .get_parameter_value().string_value
        self.map_frame = self.declare_parameter('map_frame', 'map') \
            .get_parameter_value().string_value
        self.base_link_frame = self.declare_parameter('base_link_frame', 'base_link') \
            .get_parameter_value().string_value

        # TF buffer/listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers / Subscribers
        self.pub_target_location = self.create_publisher(String, self.target_location_topic, 10)
        self.sub_command = self.create_subscription(String, self.in_topic, self._on_command, 10)

        self.get_logger().info(
            f"LocationCommandNode started: in={self.in_topic}, target_out={self.target_location_topic}, yaml={self.location_yaml_path}"
        )

    def _get_current_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_link_frame,
                rclpy.time.Time()
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Convert quaternion to yaw
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return (x, y, yaw)
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Could not transform {self.map_frame} to {self.base_link_frame}: {ex}")
            return None

    def _load_locations_from_yaml(self) -> Dict[str, Dict[str, float]]:
        if not os.path.exists(self.location_yaml_path):
            self.get_logger().warn(f"Location YAML file not found: {self.location_yaml_path}. Creating new one.")
            return {"labels": {}}
        try:
            with open(self.location_yaml_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                if data is None:
                    return {"labels": {}}
                return data
        except Exception as e:
            self.get_logger().error(f"Failed to load locations from YAML: {e}")
            return {"labels": {}}

    def _save_locations_to_yaml(self, locations_data: Dict[str, Dict[str, float]]):
        try:
            with open(self.location_yaml_path, 'w', encoding='utf-8') as f:
                yaml.dump(locations_data, f, allow_unicode=True, default_flow_style=False)
            self.get_logger().info(f"Locations saved to {self.location_yaml_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save locations to YAML: {e}")

    def _on_command(self, msg: String):
        command_str = msg.data.strip()
        self.get_logger().info(f"Received command: {command_str}")

        parts = command_str.split(':', 1)
        action = parts[0]
        target = parts[1] if len(parts) > 1 else ""

        locations_data = self._load_locations_from_yaml()
        labels = locations_data.get("labels", {})

        if action == "save":
            if not target:
                self.get_logger().warn("Save command received without a target location name.")
                return
            
            current_pose = self._get_current_pose()
            if current_pose:
                x, y, yaw = current_pose
                labels[target] = {"x": x, "y": y, "yaw": yaw}
                locations_data["labels"] = labels
                self._save_locations_to_yaml(locations_data)
                self.get_logger().info(f"Saved location '{target}' at x:{x:.2f}, y:{y:.2f}, yaw:{yaw:.2f}")
            else:
                self.get_logger().error("Failed to get current pose to save location.")
        elif action == "go":
            if not target:
                self.get_logger().warn("Go command received without a target location name.")
                return
            if target in labels:
                out_msg = String()
                out_msg.data = target
                self.pub_target_location.publish(out_msg)
                self.get_logger().info(f"Publishing target location '{target}' to {self.target_location_topic}")
            else:
                self.get_logger().warn(f"Unknown location '{target}'. Cannot go.")
        elif action == "delete":
            if not target:
                self.get_logger().warn("Delete command received without a target location name.")
                return
            if target in labels:
                del labels[target]
                locations_data["labels"] = labels
                self._save_locations_to_yaml(locations_data)
                self.get_logger().info(f"Deleted location '{target}'.")
            else:
                self.get_logger().warn(f"Location '{target}' not found. Cannot delete.")
        elif action == "list":
            if labels:
                self.get_logger().info("Available locations:")
                for name, pose in labels.items():
                    self.get_logger().info(f"  - {name}: x={pose['x']:.2f}, y={pose['y']:.2f}, yaw={pose['yaw']:.2f}")
            else:
                self.get_logger().info("No locations saved yet.")
        else:
            self.get_logger().warn(f"Unknown action '{action}' received.")


def main(args=None):
    rclpy.init(args=args)
    node = LocationCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
