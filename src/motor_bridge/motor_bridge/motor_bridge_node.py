#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import serial
import sys
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int8


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else (hi if x > hi else x)


class MotorBridge(Node):
    """
    - /turn_dir (std_msgs/Int8): -1=좌, 0=정/직진, +1=우  → 회전 명령이 최우선
    - /cmd_vel (geometry_msgs/Twist): turn_dir==0 일 때만 linear.x(및 /auto_steer)로 M1/M2 계산
    - /auto_steer (std_msgs/Float32): 조향 각도(deg). 최근 수신 시각이 steer_timeout 이내일 때만 반영
    - 시리얼 메시지: "CMD M1:<num> M2:<num>\\n"
      * int_command=True 이면 정수로 전송
      * int_command=False 이면 소수 3자리로 전송

    ※ 주의: 이제부터 M1/M2는 '물리적 rps' 단위 그대로 전송됩니다
    """

    def __init__(self) -> None:
        super().__init__('motor_bridge')

        # ------- Parameters -------
        # Serial / timing
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('serial_timeout', 0.05)
        self.declare_parameter('send_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout', 1.0)
        self.declare_parameter('steer_timeout', 1.0)

        # Startup silence (seconds)
        self.declare_parameter('startup_silence_sec', 3.0)

        # Robot geometry
        self.declare_parameter('wheel_radius', 0.25)   # [m]
        self.declare_parameter('track_width', 0.565)   # [m]
        self.declare_parameter('L_virtual', 0.60)      # [m]

        # Limits / deadbands / spin  (단위: 물리적 rps)
        self.declare_parameter('max_rps', 150.0)
        self.declare_parameter('deadband_v', 0.01)
        self.declare_parameter('deadband_deg', 0.5)
        self.declare_parameter('spin_rps', 100.0)

        # Output formatting
        self.declare_parameter('int_command', True)

        # ------- Read parameters -------
        self.port: str = self.get_parameter('port').get_parameter_value().string_value
        self.baud: int = int(self.get_parameter('baud').get_parameter_value().integer_value or 115200)
        self.serial_timeout: float = float(self.get_parameter('serial_timeout').get_parameter_value().double_value)
        self.send_rate_hz: float = float(self.get_parameter('send_rate_hz').get_parameter_value().double_value)
        self.cmd_timeout: float = float(self.get_parameter('cmd_timeout').get_parameter_value().double_value)
        self.steer_timeout: float = float(self.get_parameter('steer_timeout').get_parameter_value().double_value)

        self.startup_silence_sec: float = float(
            self.get_parameter('startup_silence_sec').get_parameter_value().double_value
        )

        self.wheel_radius: float = float(self.get_parameter('wheel_radius').get_parameter_value().double_value)
        self.track_width: float = float(self.get_parameter('track_width').get_parameter_value().double_value)
        self.L_virtual: float = float(self.get_parameter('L_virtual').get_parameter_value().double_value)

        self.max_rps: float = float(self.get_parameter('max_rps').get_parameter_value().double_value)
        self.deadband_v: float = float(self.get_parameter('deadband_v').get_parameter_value().double_value)
        self.deadband_deg: float = float(self.get_parameter('deadband_deg').get_parameter_value().double_value)
        self.spin_rps: float = float(self.get_parameter('spin_rps').get_parameter_value().double_value)
        self.int_command: bool = bool(self.get_parameter('int_command').get_parameter_value().bool_value)

        # ------- Serial -------
        self.ser: Optional[serial.Serial] = None
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.serial_timeout)
        except Exception as e:
            self.get_logger().error(f"Serial open failed on {self.port} @ {self.baud}: {e}")
            self.get_logger().error("Exiting...")
            rclpy.shutdown()
            sys.exit(1)

        # ------- Subscriptions -------
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.steer_sub = self.create_subscription(Float32, '/auto_steer', self.on_steer, 10)
        self.turn_sub = self.create_subscription(Int8, '/turn_dir', self.on_turn, 10)

        # ------- State -------
        self.last_cmd: Twist = Twist()
        self.last_cmd_time: float = 0.0
        self.last_deg: float = 0.0
        self.last_deg_time: float = 0.0
        self.last_turn: int = 0  # -1, 0, +1

        # Startup timing
        self.start_time = time.time()
        self._startup_silence_logged = False

        # ------- Periodic sender -------
        self.timer = self.create_timer(1.0 / self.send_rate_hz, self.send_command)

        # ------- Logs -------
        self.get_logger().info("MotorBridge started (/cmd_vel + /auto_steer + /turn_dir)")
        self.get_logger().info(
            f"port={self.port}, baud={self.baud}, int_command={self.int_command}, "
            f"send_rate_hz={self.send_rate_hz}, startup_silence_sec={self.startup_silence_sec}"
        )
        # 물리적 rps로 환산 정보
        self.get_logger().info(
            f"wheel_radius={self.wheel_radius} m → 1.0 m/s == {1.0 / (2 * math.pi * self.wheel_radius):.3f} rps (physical)"
        )

    # ----- Callbacks -----
    def on_cmd(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_time = time.time()
        if self.last_turn == 0:
            self.send_command()

    def on_steer(self, msg: Float32) -> None:
        self.last_deg = float(msg.data)
        self.last_deg_time = time.time()
        if self.last_turn == 0:
            self.send_command()

    def on_turn(self, msg: Int8) -> None:
        self.last_turn = int(max(-1, min(1, msg.data)))
        self.send_command()

    # ----- Kinematics -----
    def twist_steer_to_rps(self, v_mps: float, delta_deg: float) -> Tuple[float, float]:
        # 데드밴드
        if abs(v_mps) < self.deadband_v:
            v_mps = 0.0
        if abs(delta_deg) < self.deadband_deg:
            delta_deg = 0.0

        delta_rad = math.radians(delta_deg)
        omega = 0.0 if self.L_virtual <= 1e-6 else v_mps * math.tan(delta_rad) / self.L_virtual

        v_l = v_mps - 0.5 * omega * self.track_width
        v_r = v_mps + 0.5 * omega * self.track_width

        # 바퀴 선속도 -> 물리적 rps
        two_pi_r = 2.0 * math.pi * self.wheel_radius
        m1_rps = v_l / two_pi_r
        m2_rps = v_r / two_pi_r

        # 클램프 (단위: 물리적 rps)
        m1_rps = clamp(m1_rps, -self.max_rps, self.max_rps)
        m2_rps = clamp(m2_rps, -self.max_rps, self.max_rps)
        return m1_rps, m2_rps

    # ----- Sender -----
    def _format_cmd(self, m1: float, m2: float) -> str:
        if self.int_command:
            return f"CMD M1:{int(round(m1))} M2:{int(round(m2))}\n"
        return f"CMD M1:{m1:.3f} M2:{m2:.3f}\n"

    def _in_startup_silence(self) -> bool:
        """True면 아직 침묵 구간(시작 후 N초)"""
        elapsed = time.time() - self.start_time
        if elapsed < self.startup_silence_sec:
            if not self._startup_silence_logged:
                remain = self.startup_silence_sec - elapsed
                self.get_logger().info(
                    f"Startup silence... (no serial writes for {remain:.1f}s more)"
                )
                self._startup_silence_logged = True
            return True
        return False

    def send_command(self) -> None:
        # 시작 후 N초 동안은 전송하지 않음
        if self._in_startup_silence():
            return

        now = time.time()
        td = self.last_turn

        if td == -1:
            m1, m2 = +self.spin_rps, -self.spin_rps   # 물리적 rps 단위로 해석됨
        elif td == 1:
            m1, m2 = -self.spin_rps, +self.spin_rps
        else:
            if (now - self.last_cmd_time) > self.cmd_timeout:
                m1 = m2 = 0.0
            else:
                v = float(self.last_cmd.linear.x)
                delta_deg = self.last_deg if (now - self.last_deg_time) <= self.steer_timeout else 0.0
                m1, m2 = self.twist_steer_to_rps(v, delta_deg)

        cmd_str = self._format_cmd(m1, m2)

        try:
            if self.ser is not None:
                self.ser.write(cmd_str.encode('utf-8'))
                self.get_logger().info(f"Sent -> {cmd_str.strip()}")
            else:
                self.get_logger().error("Serial port not initialized.")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.ser is not None and node.ser.is_open:
                node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
