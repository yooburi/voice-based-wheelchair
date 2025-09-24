#!/usr/bin/env python3
import math, time, serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int8

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

class MotorBridge(Node):
    """
    - /turn_dir(Int8): -1=좌, 0=정/직진, +1=우  → 회전 명령 우선
    - /cmd_vel(Twist): turn_dir==0일 때만 linear.x(및 /auto_steer)로 M1/M2 계산
    - 시리얼 메시지:  CMD M1:<float> M2:<float>\\n  (TD 안 씀)
    - 타이머 주기마다 '항상' 전송(중복 억제/타임아웃 정지 없음)
    """
    def __init__(self):
        super().__init__('motor_bridge')

        # ------- Parameters -------
        self.declare_parameter('port', '/dev/ttyACM2')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('serial_timeout', 0.05)
        self.declare_parameter('send_rate_hz', 20.0)

        self.declare_parameter('steer_timeout', 1.0)

        # Robot geometry
        self.declare_parameter('wheel_radius', 0.25)   # [m]
        self.declare_parameter('track_width', 0.565)   # [m]
        self.declare_parameter('L_virtual',   0.60)    # [m]

        # Limits / deadbands / spin
        self.declare_parameter('max_rps',     150.0)
        self.declare_parameter('deadband_v',    0.01)
        self.declare_parameter('deadband_deg',  0.5)
        self.declare_parameter('spin_rps',    100.0)

        # Scale from physical rps to controller rps command
        self.declare_parameter('scale_to_rps', 100.0)

        # Read parameters
        self.port           = self.get_parameter('port').get_parameter_value().string_value
        self.baud           = int(self.get_parameter('baud').get_parameter_value().integer_value or 115200)
        self.serial_timeout = float(self.get_parameter('serial_timeout').get_parameter_value().double_value)
        self.send_rate_hz   = float(self.get_parameter('send_rate_hz').get_parameter_value().double_value)

        self.steer_timeout  = float(self.get_parameter('steer_timeout').get_parameter_value().double_value)

        self.wheel_radius   = float(self.get_parameter('wheel_radius').get_parameter_value().double_value)
        self.track_width    = float(self.get_parameter('track_width').get_parameter_value().double_value)
        self.L_virtual      = float(self.get_parameter('L_virtual').get_parameter_value().double_value)

        self.max_rps        = float(self.get_parameter('max_rps').get_parameter_value().double_value)
        self.deadband_v     = float(self.get_parameter('deadband_v').get_parameter_value().double_value)
        self.deadband_deg   = float(self.get_parameter('deadband_deg').get_parameter_value().double_value)
        self.spin_rps       = float(self.get_parameter('spin_rps').get_parameter_value().double_value)

        self.scale_to_rps   = float(self.get_parameter('scale_to_rps').get_parameter_value().double_value)

        # Serial
        self.ser = serial.Serial(self.port, self.baud, timeout=self.serial_timeout)

        # Subscriptions
        self.cmd_sub   = self.create_subscription(Twist,   '/cmd_vel',    self.on_cmd,   10)
        self.steer_sub = self.create_subscription(Float32, '/auto_steer', self.on_steer, 10)
        self.turn_sub  = self.create_subscription(Int8,    '/turn_dir',   self.on_turn,  10)

        # State
        self.last_cmd = Twist(); self.last_cmd_time = time.time()
        self.last_deg = 0.0;     self.last_deg_time = 0.0
        self.last_turn = 0       # -1, 0, +1

        # Periodic sender (항상 전송)
        self.timer = self.create_timer(1.0/self.send_rate_hz, self.send_command)

        self.get_logger().info("MotorBridge started (/cmd_vel + /auto_steer + /turn_dir, ALWAYS streaming)")
        self.get_logger().info(
            f"scale_to_rps={self.scale_to_rps}, wheel_radius={self.wheel_radius}, "
            f"1.0 m/s -> {1.0/(2*math.pi*self.wheel_radius)*self.scale_to_rps:.3f} rps"
        )

    # ----- Callbacks -----
    def on_cmd(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = time.time()

    def on_steer(self, msg: Float32):
        self.last_deg = float(msg.data)
        self.last_deg_time = time.time()

    def on_turn(self, msg: Int8):
        self.last_turn = int(max(-1, min(1, msg.data)))

    # ----- Kinematics -----
    def twist_steer_to_rps(self, v_mps: float, delta_deg: float):
        # deadbands
        if abs(v_mps) < self.deadband_v:
            v_mps = 0.0
        if abs(delta_deg) < self.deadband_deg:
            delta_deg = 0.0

        delta_rad = math.radians(delta_deg)
        omega = 0.0 if self.L_virtual <= 1e-6 else v_mps * math.tan(delta_rad) / self.L_virtual

        v_l = v_mps - 0.5 * omega * self.track_width
        v_r = v_mps + 0.5 * omega * self.track_width

        # physical rps from wheel linear velocity
        two_pi_r = 2.0 * math.pi * self.wheel_radius
        m1_rps = v_l / two_pi_r
        m2_rps = v_r / two_pi_r

        # scale up to controller's rps command space
        m1_rps *= self.scale_to_rps
        m2_rps *= self.scale_to_rps

        # clamp
        m1_rps = clamp(m1_rps, -self.max_rps, self.max_rps)
        m2_rps = clamp(m2_rps, -self.max_rps, self.max_rps)
        return m1_rps, m2_rps


    def send_command(self):
        now = time.time()

        td = self.last_turn
        if td == -1:         # left spin
            m1, m2 = +self.spin_rps, -self.spin_rps
        elif td == 1:        # right spin
            m1, m2 = -self.spin_rps, +self.spin_rps
        else:
            # 마지막 명령 유지하여 계속 전송 (타임아웃 정지 없음)
            v = float(self.last_cmd.linear.x)
            # 조향은 최근값이 일정 시간 지나면 0으로
            delta_deg = self.last_deg if (now - self.last_deg_time) <= self.steer_timeout else 0.0
            m1, m2 = self.twist_steer_to_rps(v, delta_deg)

        # ---- 전송방식만 변경: 정수로 전송 ----
        m1_i = int(round(m1))
        m2_i = int(round(m2))
        cmd_str = f"CMD M1:{m1_i} M2:{m2_i}\n"

        try:
            self.ser.write(cmd_str.encode('utf-8'))
            self.get_logger().info(f"Sent -> {cmd_str.strip()}")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")


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
