#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header

import tf2_ros
try:
    from sensor_msgs_py import point_cloud2 as pc2  # type: ignore
except Exception:  # pragma: no cover
    pc2 = None  # type: ignore


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)


def transform_point(x: float, y: float, z: float, t) -> Tuple[float, float, float]:
    # t is geometry_msgs/Transform
    tx = t.translation.x
    ty = t.translation.y
    tz = t.translation.z
    qx = t.rotation.x
    qy = t.rotation.y
    qz = t.rotation.z
    qw = t.rotation.w
    # Rotate (x,y,z) by quaternion then translate
    # Using quaternion formula: v' = q * v * q_conj
    # Expanded optimized form
    xx = qw * x + qy * z - qz * y
    yy = qw * y + qz * x - qx * z
    zz = qw * z + qx * y - qy * x
    ww = -qx * x - qy * y - qz * z
    rx = ww * -qx + xx * qw + yy * -qz - zz * -qy
    ry = ww * -qy + yy * qw + zz * -qx - xx * -qz
    rz = ww * -qz + zz * qw + xx * -qy - yy * -qx
    return (rx + tx, ry + ty, rz + tz)


def path_length_xy(points: List[Tuple[float, float]]) -> float:
    d = 0.0
    for i in range(1, len(points)):
        dx = points[i][0] - points[i - 1][0]
        dy = points[i][1] - points[i - 1][1]
        d += math.hypot(dx, dy)
    return d


def resample_polyline(points: List[Tuple[float, float]], ds: float) -> List[Tuple[float, float]]:
    if not points:
        return []
    if len(points) == 1:
        return points[:]
    ds = max(ds, 1e-3)
    # Build cumulative distances
    cum: List[float] = [0.0]
    for i in range(1, len(points)):
        dx = points[i][0] - points[i - 1][0]
        dy = points[i][1] - points[i - 1][1]
        cum.append(cum[-1] + math.hypot(dx, dy))
    total = cum[-1]
    if total < 1e-6:
        return [points[0]]
    out: List[Tuple[float, float]] = []
    s = 0.0
    j = 0
    while s <= total and j < len(points) - 1:
        # Advance to segment containing s
        while j < len(points) - 1 and cum[j + 1] < s:
            j += 1
        if j >= len(points) - 1:
            break
        seg_len = max(cum[j + 1] - cum[j], 1e-9)
        t = (s - cum[j]) / seg_len
        x = points[j][0] * (1.0 - t) + points[j + 1][0] * t
        y = points[j][1] * (1.0 - t) + points[j + 1][1] * t
        out.append((x, y))
        s += ds
    if out and (out[-1] != points[-1]):
        out.append(points[-1])
    elif not out:
        out = [points[0], points[-1]]
    return out


class LocalAvoidPlanner(Node):
    def __init__(self) -> None:
        super().__init__('local_avoid_planner')

        # Topics/frames
        self.plan_topic = self.declare_parameter('plan_topic', '/plan') \
            .get_parameter_value().string_value
        self.scan_topic = self.declare_parameter('scan_topic', '/scan') \
            .get_parameter_value().string_value
        self.points_topic = self.declare_parameter('points_topic', '/depth_camera/depth/color/points') \
            .get_parameter_value().string_value
        self.out_topic = self.declare_parameter('out_topic', '/local_plan') \
            .get_parameter_value().string_value
        self.marker_topic = self.declare_parameter('marker_topic', '/local_plan_marker') \
            .get_parameter_value().string_value
        self.global_frame = self.declare_parameter('global_frame', 'map') \
            .get_parameter_value().string_value
        self.base_frame = self.declare_parameter('base_frame', 'base_link') \
            .get_parameter_value().string_value

        # Core parameters (codex.md defaults)
        self.horizon_length = float(self.declare_parameter('horizon_length', 8.0) \
            .get_parameter_value().double_value)
        self.publish_length = float(self.declare_parameter('publish_length', 5.0) \
            .get_parameter_value().double_value)
        self.corridor_half_width = float(self.declare_parameter('corridor_half_width', 0.5) \
            .get_parameter_value().double_value)
        self.min_clear_margin = float(self.declare_parameter('min_clear_margin', 0.45) \
            .get_parameter_value().double_value)
        self.occupancy_threshold = int(self.declare_parameter('occupancy_threshold', 12) \
            .get_parameter_value().integer_value)
        self.curvature_max = float(self.declare_parameter('curvature_max', 0.6) \
            .get_parameter_value().double_value)
        self.lateral_offsets = list(self.declare_parameter('lateral_offsets', [-0.8, -0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6, 0.8]) \
            .get_parameter_value().double_array_value)
        self.path_ds = float(self.declare_parameter('path_ds', 0.1) \
            .get_parameter_value().double_value)
        self.fusion_window_sec = float(self.declare_parameter('fusion_window_sec', 0.5) \
            .get_parameter_value().double_value)
        self.scan_max_range = float(self.declare_parameter('scan_max_range', 8.0) \
            .get_parameter_value().double_value)
        self.points_voxel = float(self.declare_parameter('points_voxel', 0.10) \
            .get_parameter_value().double_value)
        self.resume_hysteresis = float(self.declare_parameter('resume_hysteresis', 0.5) \
            .get_parameter_value().double_value)
        self.resume_hold_sec = float(self.declare_parameter('resume_hold_sec', 0.8) \
            .get_parameter_value().double_value)

        # Extra filters
        self.pc_z_min = float(self.declare_parameter('points_z_min', 0.05) \
            .get_parameter_value().double_value)
        self.pc_z_max = float(self.declare_parameter('points_z_max', 2.0) \
            .get_parameter_value().double_value)
        self.rate_hz = float(self.declare_parameter('rate_hz', 10.0) \
            .get_parameter_value().double_value)
        self.viz_enable = bool(self.declare_parameter('viz_enable', True) \
            .get_parameter_value().bool_value)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # I/O
        self.plan_sub = self.create_subscription(Path, self.plan_topic, self._on_plan, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)
        self.pc_sub = self.create_subscription(PointCloud2, self.points_topic, self._on_points, 1)
        self.local_pub = self.create_publisher(Path, self.out_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        # State
        self._global_path_xy: List[Tuple[float, float]] = []
        self._obs: List[Tuple[float, float, float]] = []  # (x, y, stamp_sec)
        self._state = 'TRACK'
        self._resume_ok_since: Optional[float] = None

        self.timer = self.create_timer(1.0 / max(self.rate_hz, 1.0), self._on_timer)
        self.get_logger().info(
            f"LocalAvoidPlanner up: in plan={self.plan_topic}, scan={self.scan_topic}, points={self.points_topic}, out={self.out_topic}"
        )

    # -------------------- Subscribers --------------------
    def _on_plan(self, msg: Path) -> None:
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not pts:
            self._global_path_xy = []
            return
        # Optional resample for uniform spacing
        self._global_path_xy = resample_polyline(pts, max(self.path_ds, 0.05))

    def _on_scan(self, msg: LaserScan) -> None:
        # Convert scan to points in map frame
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        frame = msg.header.frame_id or self.base_frame
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, frame, rclpy.time.Time.from_msg(msg.header.stamp), timeout=Duration(seconds=0.2)
            )
        except Exception:
            try:
                tf = self.tf_buffer.lookup_transform(self.global_frame, frame, Time(), timeout=Duration(seconds=0.1))
            except Exception:
                return

        angle = msg.angle_min
        step = max(int(len(msg.ranges) / 720), 1)  # downsample beams if very dense
        for i, r in enumerate(msg.ranges):
            if i % step != 0:
                angle += msg.angle_increment
                continue
            if not math.isfinite(r) or r <= 0.0 or r > self.scan_max_range:
                angle += msg.angle_increment
                continue
            x_l = r * math.cos(angle)
            y_l = r * math.sin(angle)
            mx, my, _ = transform_point(x_l, y_l, 0.0, tf.transform)
            self._obs.append((mx, my, now_sec))
            angle += msg.angle_increment

    def _on_points(self, msg: PointCloud2) -> None:
        if pc2 is None:
            return
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        frame = msg.header.frame_id or self.base_frame
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, frame, rclpy.time.Time.from_msg(msg.header.stamp), timeout=Duration(seconds=0.2)
            )
        except Exception:
            try:
                tf = self.tf_buffer.lookup_transform(self.global_frame, frame, Time(), timeout=Duration(seconds=0.1))
            except Exception:
                return

        # Voxel downsample in global frame
        vox = max(self.points_voxel, 0.05)
        occ = set()
        count = 0
        for pt in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = float(pt[0]), float(pt[1]), float(pt[2])
            if not (self.pc_z_min <= z <= self.pc_z_max):
                continue
            mx, my, mz = transform_point(x, y, z, tf.transform)
            # Use voxel grid key
            kx = int(math.floor(mx / vox))
            ky = int(math.floor(my / vox))
            key = (kx, ky)
            if key in occ:
                continue
            occ.add(key)
            self._obs.append((mx, my, now_sec))
            count += 1
            if count >= 400:  # cap per cloud
                break

    # -------------------- Core timer --------------------
    def _on_timer(self) -> None:
        # Prune old obstacle points
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        win = max(self.fusion_window_sec, 0.05)
        self._obs = [(x, y, t) for (x, y, t) in self._obs if (now_sec - t) <= win]

        # If no global path, do nothing
        if not self._global_path_xy:
            return

        pose = self._lookup_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose

        # Extract segment ahead of robot position
        seg = self._extract_segment(rx, ry, self.horizon_length)
        if len(seg) < 2:
            return

        # Corridor occupancy and clearance
        occ_count, min_clear = self._corridor_occupancy(seg)

        # FSM transitions
        if self._state == 'TRACK':
            if (occ_count >= self.occupancy_threshold) or (min_clear < self.min_clear_margin):
                self._state = 'AVOID'
                self._resume_ok_since = None
        else:  # AVOID
            if occ_count <= int(self.resume_hysteresis * self.occupancy_threshold):
                if self._resume_ok_since is None:
                    self._resume_ok_since = now_sec
                elif (now_sec - self._resume_ok_since) >= self.resume_hold_sec:
                    self._state = 'TRACK'
                    self._resume_ok_since = None
            else:
                self._resume_ok_since = None

        # Plan selection
        chosen: List[Tuple[float, float]]
        if self._state == 'AVOID':
            chosen = self._avoid_path(seg)
        else:
            chosen = seg

        # Truncate to publish_length
        chosen = self._truncate_length(chosen, self.publish_length)

        # Publish local plan
        path_msg = self._to_path(chosen)
        self.local_pub.publish(path_msg)

        # Markers
        if self.viz_enable:
            self._publish_markers(chosen)

    # -------------------- Helpers --------------------
    def _lookup_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, Time(), timeout=Duration(seconds=0.2)
            )
        except Exception:
            return None
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = quat_to_yaw(tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w)
        return (x, y, yaw)

    def _nearest_index(self, x: float, y: float) -> int:
        best_i = 0
        best_d = float('inf')
        for i, (px, py) in enumerate(self._global_path_xy):
            d = (px - x) * (px - x) + (py - y) * (py - y)
            if d < best_d:
                best_d = d
                best_i = i
        return best_i

    def _extract_segment(self, x: float, y: float, length_m: float) -> List[Tuple[float, float]]:
        if not self._global_path_xy:
            return []
        start = self._nearest_index(x, y)
        pts: List[Tuple[float, float]] = [self._global_path_xy[start]]
        acc = 0.0
        for i in range(start + 1, len(self._global_path_xy)):
            px, py = self._global_path_xy[i - 1]
            qx, qy = self._global_path_xy[i]
            acc += math.hypot(qx - px, qy - py)
            pts.append((qx, qy))
            if acc >= max(length_m, 0.5):
                break
        # Ensure reasonable density
        return resample_polyline(pts, max(self.path_ds, 0.05))

    def _corridor_occupancy(self, seg: List[Tuple[float, float]]) -> Tuple[int, float]:
        # Precompute segment cumulative and tangents
        cum: List[float] = [0.0]
        tang: List[Tuple[float, float]] = []
        for i in range(1, len(seg)):
            dx = seg[i][0] - seg[i - 1][0]
            dy = seg[i][1] - seg[i - 1][1]
            L = math.hypot(dx, dy)
            cum.append(cum[-1] + L)
            if L > 1e-9:
                tang.append((dx / L, dy / L))
            else:
                tang.append((1.0, 0.0))
        total = cum[-1] if cum else 0.0

        def proj_point(px: float, py: float) -> Tuple[float, float]:
            # returns (s along path, signed lateral d)
            best = (0.0, float('inf'), 0, 0.0)  # (s, abs_d, seg_i, signed_d)
            for i in range(len(seg) - 1):
                ax, ay = seg[i]
                bx, by = seg[i + 1]
                vx = bx - ax
                vy = by - ay
                L2 = vx * vx + vy * vy
                if L2 < 1e-12:
                    continue
                ux = px - ax
                uy = py - ay
                t = clamp((ux * vx + uy * vy) / L2, 0.0, 1.0)
                qx = ax + vx * t
                qy = ay + vy * t
                dx = px - qx
                dy = py - qy
                d = math.hypot(dx, dy)
                # Signed lateral using 2D cross product sign
                cross = vx * (py - ay) - vy * (px - ax)
                sign = 1.0 if cross > 0.0 else -1.0
                s = cum[i] + math.sqrt(max(0.0, (qx - ax) ** 2 + (qy - ay) ** 2))
                if d < best[1]:
                    best = (s, d, i, sign * d)
            return (best[0], best[3])

        occ = 0
        min_clear = float('inf')
        for (ox, oy, _) in self._obs:
            s, d = proj_point(ox, oy)
            if 0.0 <= s <= total:
                if abs(d) <= self.corridor_half_width:
                    occ += 1
                min_clear = min(min_clear, abs(d))
        if min_clear == float('inf'):
            min_clear = 999.0
        return occ, min_clear

    def _offset_path(self, seg: List[Tuple[float, float]], offset: float) -> List[Tuple[float, float]]:
        if abs(offset) < 1e-6:
            return seg[:]
        out: List[Tuple[float, float]] = []
        n = len(seg)
        for i in range(n):
            # Compute local tangent using neighbors
            if i == 0:
                vx = seg[i + 1][0] - seg[i][0]
                vy = seg[i + 1][1] - seg[i][1]
            elif i == n - 1:
                vx = seg[i][0] - seg[i - 1][0]
                vy = seg[i][1] - seg[i - 1][1]
            else:
                vx = seg[i + 1][0] - seg[i - 1][0]
                vy = seg[i + 1][1] - seg[i - 1][1]
            L = math.hypot(vx, vy)
            if L < 1e-9:
                nx, ny = 0.0, 0.0
            else:
                tx, ty = vx / L, vy / L
                nx, ny = -ty, tx  # left normal
            out.append((seg[i][0] + nx * offset, seg[i][1] + ny * offset))
        return out

    def _path_curvature_ok(self, pts: List[Tuple[float, float]]) -> Tuple[bool, float, float]:
        if len(pts) < 3:
            return True, 0.0, 0.0
        max_k = 0.0
        sum_k = 0.0
        cnt = 0
        for i in range(1, len(pts) - 1):
            ax, ay = pts[i - 1]
            bx, by = pts[i]
            cx, cy = pts[i + 1]
            v1x, v1y = bx - ax, by - ay
            v2x, v2y = cx - bx, cy - by
            L1 = math.hypot(v1x, v1y)
            L2 = math.hypot(v2x, v2y)
            if L1 < 1e-6 or L2 < 1e-6:
                continue
            a1 = math.atan2(v1y, v1x)
            a2 = math.atan2(v2y, v2x)
            da = abs((a2 - a1 + math.pi) % (2 * math.pi) - math.pi)
            ds = 0.5 * (L1 + L2)
            k = da / max(ds, 1e-6)
            max_k = max(max_k, k)
            sum_k += k
            cnt += 1
        avg_k = (sum_k / cnt) if cnt > 0 else 0.0
        return (max_k <= self.curvature_max), max_k, avg_k

    def _min_clearance(self, pts: List[Tuple[float, float]]) -> float:
        if not self._obs:
            return 999.0
        best = float('inf')
        # Limit obstacle set for cost speed
        obst = self._obs
        if len(obst) > 800:
            step = max(int(len(obst) / 800), 1)
            obst = obst[::step]
        for (px, py) in pts:
            for (ox, oy, _) in obst:
                d = math.hypot(px - ox, py - oy)
                if d < best:
                    best = d
        return best

    def _avoid_path(self, seg: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        best_cost = float('inf')
        best: Optional[List[Tuple[float, float]]] = None
        # We prefer small offsets; iterate offsets near zero first
        offsets = sorted(self.lateral_offsets, key=lambda v: abs(v))
        for off in offsets:
            cand = self._offset_path(seg, off)
            ok, max_k, avg_k = self._path_curvature_ok(cand)
            if not ok:
                continue
            min_clr = self._min_clearance(cand)
            if min_clr < self.min_clear_margin:
                continue
            # Cost: offset magnitude + avg curvature + inverse clearance
            cost = 1.0 * abs(off) + 1.0 * avg_k + 0.8 * (1.0 / max(min_clr, 1e-3))
            if cost < best_cost:
                best_cost = cost
                best = cand
        if best is not None:
            return best
        # Failsafe: produce a very short straight path ahead from first point
        return self._stop_path(seg)

    def _stop_path(self, seg: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        if len(seg) < 2:
            return seg[:]
        a = seg[0]
        b = seg[1]
        vx, vy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(vx, vy)
        if L < 1e-6:
            return [a, a]
        tx, ty = vx / L, vy / L
        stop_len = 0.2
        return [a, (a[0] + tx * stop_len, a[1] + ty * stop_len)]

    def _truncate_length(self, pts: List[Tuple[float, float]], Lmax: float) -> List[Tuple[float, float]]:
        if not pts:
            return []
        if len(pts) == 1:
            return pts[:]
        out = [pts[0]]
        acc = 0.0
        for i in range(1, len(pts)):
            ax, ay = out[-1]
            bx, by = pts[i]
            d = math.hypot(bx - ax, by - ay)
            if acc + d <= Lmax:
                out.append((bx, by))
                acc += d
            else:
                r = clamp((Lmax - acc) / max(d, 1e-9), 0.0, 1.0)
                out.append((ax + (bx - ax) * r, ay + (by - ay) * r))
                break
        return resample_polyline(out, max(self.path_ds, 0.05))

    def _to_path(self, pts: List[Tuple[float, float]]) -> Path:
        msg = Path()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame
        for (x, y) in pts:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.position.z = 0.0
            # Orientation along path if possible
            yaw = 0.0
            if len(msg.poses) > 0:
                px = msg.poses[-1].pose.position.x
                py = msg.poses[-1].pose.position.y
                yaw = math.atan2(y - py, x - px)
            qx, qy, qz, qw = yaw_to_quat(yaw)
            p.pose.orientation.x = qx
            p.pose.orientation.y = qy
            p.pose.orientation.z = qz
            p.pose.orientation.w = qw
            msg.poses.append(p)
        return msg

    def _publish_markers(self, chosen: List[Tuple[float, float]]) -> None:
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Local path marker
        m_path = Marker()
        m_path.header.frame_id = self.global_frame
        m_path.header.stamp = now
        m_path.ns = 'local_avoid'
        m_path.id = 0
        m_path.type = Marker.LINE_STRIP
        m_path.action = Marker.ADD
        m_path.scale.x = 0.05
        m_path.color.a = 1.0
        m_path.color.r = 0.1
        m_path.color.g = 1.0
        m_path.color.b = 0.2
        m_path.lifetime = Duration(seconds=0.5).to_msg()
        m_path.points = []
        for (x, y) in chosen:
            pt = Point()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = 0.05
            m_path.points.append(pt)
        arr.markers.append(m_path)

        # Obstacles as points
        m_obs = Marker()
        m_obs.header.frame_id = self.global_frame
        m_obs.header.stamp = now
        m_obs.ns = 'local_avoid'
        m_obs.id = 1
        m_obs.type = Marker.POINTS
        m_obs.action = Marker.ADD
        m_obs.scale.x = 0.06
        m_obs.scale.y = 0.06
        m_obs.color.a = 0.9
        m_obs.color.r = 1.0
        m_obs.color.g = 0.2
        m_obs.color.b = 0.2
        m_obs.lifetime = Duration(seconds=0.5).to_msg()
        # Limit number of points visualized
        pts = self._obs
        if len(pts) > 1000:
            step = max(int(len(pts) / 1000), 1)
            pts = pts[::step]
        for (x, y, _) in pts:
            pt = Point()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = 0.05
            m_obs.points.append(pt)
        arr.markers.append(m_obs)

        self.marker_pub.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalAvoidPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
