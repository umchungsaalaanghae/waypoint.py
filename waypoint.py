#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, Imu

class SquareNavigator(Node):
    def __init__(self):
        super().__init__('square_navigator')

        self.side_len = 0.001
        self.speed = 2000.0
        self.turn_speed = 2000.0
        self.yaw_tolerance = 5.0 * math.pi/180 # 오차허용 범위
        self.threshold = 0.5 # 50cm이내로 가까워지면 도착했다 간주

        self.waypoint = None # 추가한 거
        self.lat = self.lon = self.yaw =None #lat위도, lon경도
        self.start_lat = self.start_lon = self.start_yaw = None
        self.state = "MOVE"
        self.turn_count = 0
        self.target_yaw = None

        self.mode = "SQUARE"

        self.left_pub  = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

    def gps_callback(self, msg: NavSatFix):
        self.lat = msg.latitude
        self.lon = msg.longitude

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def waypoint_callback(self, msg):
        self.waypoint = (msg.latitude, msg.longitude)

    def control_loop(self): # 자기 위치 확인, 미래 행동 판단
        if None in (self.lat, self.lon, self.yaw):
            return  # 아직 센서 정보 안 들어왔으면 아무 것도 안 함

        if self.waypoint is not None and self.mode != "WAYPOINT": # 경유점있고, waypoint가 아닐때
            self.mode = "WAYPOINT"
            self.state = "MOVE"  # 상태 초기화

        if self.mode == "SQUARE":
            self.square_motion()
        elif self.mode == "WAYPOINT":
            self.waypoint_motion()

    def square_motion(self):    
        if self.state == "MOVE":
            # 직선 시작점 기록
            if self.start_lat is None:
                self.start_lat, self.start_lon = self.lat, self.lon
        
            # 이동 거리 계산
            dx = self.lon - self.start_lon
            dy = self.lat - self.start_lat
            dist = math.hypot(dx, dy)

            if dist >= self.side_len:           # 한 변 주행 완료
                self.stop()
                self.state = "TURN"
                self.target_yaw = self.norm_angle(self.yaw + math.pi/2)  # +90도
            else:
                self.move_forward()

        elif self.state == "TURN":
            yaw_err = self.norm_angle(self.target_yaw - self.yaw)
            if abs(yaw_err) < self.yaw_tolerance:        # 회전 완료
                self.stop()
                self.turn_count += 1
                if self.turn_count >= 4:           # 사각형 완주
                    self.state = "STOP"
                else:                              # 다음 변 시작
                    self.state = "MOVE"
                    self.start_lat = self.lat     # 새 변 기준점 갱신
                    self.start_lon = self.lon
            else:
                self.rotate_in_place(yaw_err)
        
        elif self.state == "STOP":
            self.stop()        

    def waypoint_motion(self):     
        if self.waypoint is not None:
            dx = self.waypoint[1] - self.start_lon
            dy = self.waypoint[0] - self.start_lat
            dist = math.hypot(dx, dy)
            
            if dist >= self.side_len:
                self.stop()
                self.waypoint = None # 다음 경유점 기다림
                self.mode = "SQUARE"
                self.state = "MOVE"
                self.start_lat = self.lat  
                self.start_lon = self.lon
            else:
                self.move_forward()

    def move_forward(self):
        thrust = Float64()
        thrust.data = self.speed
        self.left_pub.publish(thrust)
        self.right_pub.publish(thrust)

    def rotate_in_place(self, yaw_error):
        turn_speed = 2000.0  # 양쪽 반대로 돌리는 추력값

        if yaw_error > 0: # 왼쪽 회전
            self.left_pub.publish(Float64(data=-turn_speed))
            self.right_pub.publish(Float64(data=turn_speed))
        else: # 오른쪽 회전
            self.left_pub.publish(Float64(data=turn_speed))
            self.right_pub.publish(Float64(data=-turn_speed))
    
    def stop(self):
        self.left_pub.publish(Float64(data=0.0))
        self.right_pub.publish(Float64(data=0.0))

    def norm_angle(self, angle): # (ㅠ ~ -ㅠ)
        return math.atan2(math.sin(angle), math.cos(angle))

    def process(self):
        if self.lat is None or self.lon is None:
            print("no data")
            return # 센서 값 없을 때 에러 방지

def main(args=None): # args=명령줄에서 프로그램 실행 시 같이 넘겨주는 값
    rclpy.init(args=args) # ros2 초기화
    node = SquareNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
