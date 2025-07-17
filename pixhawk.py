#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import RCIn
import serial
from collections import deque


class PixhawkControl(Node):
    def __init__(self):
        super().__init__('pixhawk_control')

        try:
            self.serial_conn = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
            self.get_logger().info('✅ Serial connected.')
        except Exception as e:
            self.serial_conn = None
            self.get_logger().warn(f'⚠️ Serial connection failed: {e}')


        self.rc_pub = self.create_publisher(UInt16MultiArray, '/mavros/rc/override', 10)
        self.vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.att_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_attitude/target_attitude', 10)

        self.thr_min = 1250
        self.thr_max = 1750
        self.thr_min_auto = 1100
        self.thr_max_auto = 1900

        # 초기값
        self.port_pwm = 1500
        self.stbd_pwm = 1500

        self.buffer_size = 5 # 최근 5개의 PWM 값을 저장하는 버퍼
        self.port_history = deque([1500] * self.buffer_size, maxlen=self.buffer_size)
        self.stbd_history = deque([1500] * self.buffer_size, maxlen=self.buffer_size)

        self.mode = 0  # 기본: 정지 모드

        # 0.1초마다 RC override 전송
        self.timer = self.create_timer(0.1, self.send_override)
        self.rc_sub = self.create_subscription(RCIn, '/mavros/rc/in', self.rc_input_callback, 10)
        self.serial_timer = self.create_timer(0.1, self.read_serial_data)  # 10Hz


    def rc_input_callback(self, msg):
        ch = msg.channels

        # CH6 → 모드 전환
        ch6 = ch[5] if len(ch) > 5 else 1500
        if ch6 < 1200:
            self.mode = 0  # 정지
        elif ch6 < 1600:
            self.mode = 2  # 자율주행
        else:
            self.mode = 1  # RC 수동

        ch5 = ch[4] if len(ch) > 4 else 1500 # CH5 → 앞/뒤
        ch4 = ch[3] if len(ch) > 3 else 1500 # CH4 → 좌/우

        # 기본 정지
        self.port_pwm = 1500
        self.stbd_pwm = 1500

        if self.mode == 2:  # 자율주행일 때만 PWM 제어
            if ch5 > 1600:
                self.port_pwm = self.stbd_pwm = 1700  # 앞으로
            elif ch5 < 1200:
                self.port_pwm = self.stbd_pwm = 1300  # 뒤로

            if ch4 > 1600:
                self.port_pwm = 1700
                self.stbd_pwm = 1500  # 오른쪽
            elif ch4 < 1200:
                self.port_pwm = 1500
                self.stbd_pwm = 1700  # 왼쪽

    def read_serial_data(self):
        if self.serial_conn and self.serial_conn.in_waiting > 0:
            try:
                line = self.serial_conn.readline().decode('utf-8').strip()
                if "," in line:
                    parts = line.split(',')
                    if len(parts) >= 2:
                        port = int(parts[0])
                        stbd = int(parts[1])
                        
                        # 평균 필터에 추가
                        self.port_history.append(port)
                        self.stbd_history.append(stbd)

                        self.port_pwm = int(sum(self.port_history) / len(self.port_history))
                        self.stbd_pwm = int(sum(self.stbd_history) / len(self.stbd_history))

            except Exception as e:
                pass

    def send_override(self):
        msg = UInt16MultiArray()

        if self.mode == 0:
            self.port_pwm = 1500
            self.stbd_pwm = 1500
            self.emergency_stop = True
            self.get_logger().warn("🛑 Emergency Stop Mode Activated")

            if self.serial_conn:
                self.serial_conn.write(b"LED,RED\n")

        elif self.mode == 2:
            self.get_logger().info("🎮 RC Mode")

            if self.serial_conn:
                self.serial_conn.write(b"LED,GREEN\n")

        elif self.mode == 1:
            self.get_logger().info("🤖 Auto Mode")
            # PWM 값 클램핑
            self.port_pwm = max(self.thr_min_auto, min(self.port_pwm, self.thr_max_auto))
            self.stbd_pwm = max(self.thr_min_auto, min(self.stbd_pwm, self.thr_max_auto))

            if self.serial_conn:
                self.serial_conn.write(b"LED,YELLOW\n")

    def send_velocity(self, lin_x=0.5, ang_z=0.0):
        msg = Twist()
        msg.linear.x = lin_x  # 앞쪽으로 0.5 m/s
        msg.angular.z = ang_z  # 회전 속도
        self.vel_pub.publish(msg)
        self.get_logger().info(f'Sent velocity: linear.x={lin_x}, angular.z={ang_z}')

    def send_position(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        self.pos_pub.publish(msg)
        self.get_logger().info(f'Sent position: x={x}, y={y}, z={z}')

    def send_attitude(self, w=1.0, x=0.0, y=0.0, z=0.0):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.orientation.w = w
        msg.pose.orientation.x = x
        msg.pose.orientation.y = y
        msg.pose.orientation.z = z
        self.att_pub.publish(msg)
        self.get_logger().info(f'Sent attitude: w={w}, x={x}, y={y}, z={z}')

    def arm_vehicle(self, arm=True): # 자율주행코드로 부팅 후 자동 시동
        client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /mavros/cmd/arming service...')

        req = CommandBool.Request()
        req.value = arm
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f"Vehicle {'armed' if arm else 'disarmed'} successfully.")
        else:
            self.get_logger().error(f"Failed to {'arm' if arm else 'disarm'} vehicle.")

        # PWM 값 제한
        port = max(self.thr_min_auto, min(self.port_pwm, self.thr_max_auto))
        stbd = max(self.thr_min_auto, min(self.stbd_pwm, self.thr_max_auto))

        # RC 채널 설정 (CH1: port, CH2: stbd)
        msg = UInt16MultiArray()
        msg.data = [port, stbd, 0, 0, 0, 0, 0, 0]

        self.rc_pub.publish(msg)
        self.get_logger().info(f'PWM Sent - Port: {port}, Stbd: {stbd}')

def main(args=None):
    rclpy.init(args=args)
    node = PixhawkControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
