#!/usr/bin/env python3
import math, time, serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int8

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')

        # ===== 파라미터 =====
        self.port            = '/dev/ttyACM0'
        self.baud            = 115200
        self.serial_timeout  = 0.05
        self.send_rate_hz    = 20.0
        self.cmd_timeout     = 0.5
        self.steer_timeout   = 1.0
        self.wheel_radius    = 0.25       # [m]
        self.track_width     = 0.565        # [m]
        self.L_virtual       = 0.60         # [m]
        self.max_rps         = 200.0
        self.deadband_v      = 0.01
        self.deadband_deg    = 0.5
        self.spin_rps        = 50.0         # turn_dir 회전 속도

        # ===== 시리얼 =====
        self.ser = serial.Serial(self.port, self.baud, timeout=self.serial_timeout)

        # ===== 구독 =====
        self.cmd_sub   = self.create_subscription(Twist,   '/cmd_vel',     self.on_cmd,   10)
        self.steer_sub = self.create_subscription(Float32, '/auto_steer',  self.on_steer, 10)
        self.turn_sub  = self.create_subscription(Int8,    '/turn_dir',    self.on_turn,  10)

        # ===== 상태 =====
        self.last_cmd = Twist(); self.last_cmd_time = 0.0
        self.last_deg = 0.0;     self.last_deg_time = 0.0
        self.last_turn = 0       # -1,0,+1

        # 주기 전송
        self.timer = self.create_timer(1.0/self.send_rate_hz, self.send_command)

        self.get_logger().info("MotorBridge started (/cmd_vel + /auto_steer + /turn_dir)")

    def on_cmd(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = time.time()
        # turn_dir==0 일 때만 반영
        if self.last_turn == 0:
            self.send_command()

    def on_steer(self, msg: Float32):
        self.last_deg = float(msg.data)
        self.last_deg_time = time.time()
        # turn_dir==0 일 때만 반영
        if self.last_turn == 0:
            self.send_command()

    def on_turn(self, msg: Int8):
        self.last_turn = int(max(-1, min(1, msg.data)))
        # turn_dir가 바뀌면 바로 반영
        self.send_command()

    def twist_steer_to_rps(self, v, delta_deg):
        if abs(v) < self.deadband_v:
            v = 0.0
        if abs(delta_deg) < self.deadband_deg:
            delta_deg = 0.0

        delta_rad = math.radians(delta_deg)
        omega = 0.0 if self.L_virtual <= 1e-6 else v * math.tan(delta_rad) / self.L_virtual

        v_l = v - 0.5 * omega * self.track_width
        v_r = v + 0.5 * omega * self.track_width

        two_pi_r = 2.0 * math.pi * self.wheel_radius
        m1_rps = clamp(v_l / two_pi_r, -self.max_rps, self.max_rps)
        m2_rps = clamp(v_r / two_pi_r, -self.max_rps, self.max_rps)
        return m1_rps, m2_rps

    def send_command(self):
        now = time.time()

        # === turn_dir 우선 처리 ===
        td = self.last_turn
        if td == -1:   # 좌회전
            m1, m2 = +self.spin_rps, -self.spin_rps
        elif td == 1:  # 우회전
            m1, m2 = -self.spin_rps, +self.spin_rps
        else:
            # === turn_dir == 0 → /cmd_vel & /auto_steer 기반 ===
            if (now - self.last_cmd_time) > self.cmd_timeout:
                m1 = m2 = 0.0
            else:
                v = float(self.last_cmd.linear.x)
                delta_deg = self.last_deg if (now - self.last_deg_time) <= self.steer_timeout else 0.0
                m1, m2 = self.twist_steer_to_rps(v, delta_deg)

        # === 아두이노 전송 ===
        cmd_str = f"CMD TD:{td} M1:{m1:.3f} M2:{m2:.3f}\n"
        try:
            self.ser.write(cmd_str.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")
        self.get_logger().info(f"Sent -> {cmd_str.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
