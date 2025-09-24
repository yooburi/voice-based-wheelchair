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
    동작 규칙
    - /turn_dir (Int8): -1=좌스핀, 0=일반주행, +1=우스핀
      * turn_dir != 0 이면, /cmd_vel, /auto_steer 는 **완전히 무시**하고
        고정 rps 쌍을 전송한다. (-1 -> +50,-50 / +1 -> -50,+50)
      * turn_dir == 0 이면, /cmd_vel.linear.x (m/s)와 /auto_steer(deg)를 사용하여
        좌우 바퀴 rps(물리적 rps)로 변환해 전송한다.

    - 시리얼 메시지 포맷: "CMD M1:<num> M2:<num>\\n"
      * int_command=False 이면 소수(기본 3자리), True면 정수로 전송
      * tx_scale: 송신 전에 곱하는 스케일 (아두이노가 ×100 할 때, 여기선 0.01이 알맞음)
    """

    def __init__(self) -> None:
        super().__init__('motor_bridge')

        # ------- Parameters -------
        # Serial / timing
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('serial_timeout', 0.05)
        self.declare_parameter('send_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout', 1.0)      # cmd_vel 타임아웃
        self.declare_parameter('steer_timeout', 1.0)    # auto_steer 타임아웃

        # Startup silence (seconds)
        self.declare_parameter('startup_silence_sec', 3.0)

        # Robot geometry
        self.declare_parameter('wheel_radius', 0.25)   # [m]
        self.declare_parameter('track_width', 0.565)   # [m]
        self.declare_parameter('L_virtual', 0.60)      # [m]

        # Limits / deadbands (단위: 물리적 rps)
        self.declare_parameter('max_rps', 250.0)
        self.declare_parameter('deadband_v', 0.01)     # m/s
        self.declare_parameter('deadband_deg', 0.5)    # deg

        # Spin 모드 고정 rps (절댓값)
        self.declare_parameter('spin_rps_fixed', 0.05)  # 요구사항: ±50

        # Output formatting
        self.declare_parameter('int_command', False)     # ★ 소수 전송 기본
        self.declare_parameter('float_precision', 3)     # ★ 소수 자릿수
        self.declare_parameter('tx_scale', 1)         # ★ 송신 스케일(아두이노 ×100과 짝)

        # ------- Read parameters -------
        gp = self.get_parameter
        self.port: str = gp('port').value
        self.baud: int = int(gp('baud').value or 115200)
        self.serial_timeout: float = float(gp('serial_timeout').value)
        self.send_rate_hz: float = float(gp('send_rate_hz').value)
        self.cmd_timeout: float = float(gp('cmd_timeout').value)
        self.steer_timeout: float = float(gp('steer_timeout').value)

        self.startup_silence_sec: float = float(gp('startup_silence_sec').value)

        self.wheel_radius: float = float(gp('wheel_radius').value)
        self.track_width: float = float(gp('track_width').value)
        self.L_virtual: float = float(gp('L_virtual').value)

        self.max_rps: float = float(gp('max_rps').value)
        self.deadband_v: float = float(gp('deadband_v').value)
        self.deadband_deg: float = float(gp('deadband_deg').value)
        self.spin_rps_fixed: float = float(gp('spin_rps_fixed').value)

        self.int_command: bool = bool(gp('int_command').value)
        self.float_precision: int = int(gp('float_precision').value)
        self.tx_scale: float = float(gp('tx_scale').value)

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
            f"float_precision={self.float_precision}, tx_scale={self.tx_scale}, "
            f"send_rate_hz={self.send_rate_hz}, startup_silence_sec={self.startup_silence_sec}"
        )
        self.get_logger().info(
            f"wheel_radius={self.wheel_radius} m → 1.0 m/s == {1.0 / (2 * math.pi * self.wheel_radius):.3f} rps"
        )
        self.get_logger().info(f"spin_rps_fixed={self.spin_rps_fixed} rps (|turn|=1일 때 고정 출력)")

    # ----- Callbacks -----
    def on_cmd(self, msg: Twist) -> None:
        if self.last_turn == 0:
            self.last_cmd = msg
            self.last_cmd_time = time.time()
            self.send_command()

    def on_steer(self, msg: Float32) -> None:
        if self.last_turn == 0:
            self.last_deg = float(msg.data)
            self.last_deg_time = time.time()
            self.send_command()

    def on_turn(self, msg: Int8) -> None:
        self.last_turn = int(max(-1, min(1, msg.data)))
        self.send_command()

    # ----- Kinematics -----
    def twist_steer_to_rps(self, v_mps: float, delta_deg: float) -> Tuple[float, float]:
        if abs(v_mps) < self.deadband_v:
            v_mps = 0.0
        if abs(delta_deg) < self.deadband_deg:
            delta_deg = 0.0

        delta_deg = -delta_deg           # ★ 조향 각도 부호 반전 (여기 한 줄만 추가)

        delta_rad = math.radians(delta_deg)
        omega = 0.0 if self.L_virtual <= 1e-6 else v_mps * math.tan(delta_rad) / self.L_virtual

        v_l = v_mps - 0.5 * omega * self.track_width
        v_r = v_mps + 0.5 * omega * self.track_width

        two_pi_r = 2.0 * math.pi * self.wheel_radius
        m1_rps = v_l / two_pi_r
        m2_rps = v_r / two_pi_r

        m1_rps = clamp(m1_rps, -self.max_rps, self.max_rps)
        m2_rps = clamp(m2_rps, -self.max_rps, self.max_rps)
        return m1_rps, m2_rps

    # ----- Sender -----
    def _format_cmd(self, m1: float, m2: float) -> str:
        """
        전송 직전에 tx_scale을 곱해 문자열 생성.
        - 아두이노가 수신 후 CMD_SCALE(예: 100)을 다시 곱해 최종 rps로 사용.
        """
        m1_tx = m1 * self.tx_scale
        m2_tx = m2 * self.tx_scale
        if self.int_command:
            return f"CMD M1:{int(round(m1_tx))} M2:{int(round(m2_tx))}\n"
        fmt = f"{{:.{self.float_precision}f}}"
        return f"CMD M1:{fmt.format(m1_tx)} M2:{fmt.format(m2_tx)}\n"

    def _in_startup_silence(self) -> bool:
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
        if self._in_startup_silence():
            return

        now = time.time()
        td = self.last_turn

        if td == -1:
            m1, m2 = +self.spin_rps_fixed, -self.spin_rps_fixed
        elif td == 1:
            m1, m2 = -self.spin_rps_fixed, +self.spin_rps_fixed
        else:
            if (now - self.last_cmd_time) > self.cmd_timeout:
                m1 = m2 = 0.0
            else:
                v = float(self.last_cmd.linear.x)  # m/s
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
