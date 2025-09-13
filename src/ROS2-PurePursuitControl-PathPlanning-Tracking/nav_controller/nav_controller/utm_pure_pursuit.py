#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
import scipy.interpolate as si
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time

# Pure Pursuit 파라미터 (control.py와 동일한 구조)
lookahead_distance = 0.3  # UTM 좌표 기준 (미터)
speed = 35.0            # 기본 속도 (m/s)

def euler_from_quaternion(x, y, z, w):
    """쿼터니언에서 yaw 각도 추출 (control.py와 동일)"""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def bspline_planning(array, sn):
    """B-Spline 경로 스무딩 (control.py와 동일)"""
    try:
        array = np.array(array)
        x = array[:, 0]
        y = array[:, 1]
        N = 2
        t = range(len(x))
        x_tup = si.splrep(t, x, k=N)
        y_tup = si.splrep(t, y, k=N)

        x_list = list(x_tup)
        xl = x.tolist()
        x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

        y_list = list(y_tup)
        yl = y.tolist()
        y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

        ipl_t = np.linspace(0.0, len(x) - 1, sn)
        rx = si.splev(ipl_t, x_list)
        ry = si.splev(ipl_t, y_list)
        path = [(rx[i], ry[i]) for i in range(len(rx))]
    except:
        path = array
    return path

def pure_pursuit(current_x, current_y, current_heading, path, index):
    """Pure Pursuit 알고리즘 (control.py와 동일한 로직)"""
    global lookahead_distance
    closest_point = None
    v = speed
    
    # lookahead_distance보다 먼 경로점 찾기
    for i in range(index, len(path)):
        x = path[i][0]
        y = path[i][1]
        distance = math.hypot(current_x - x, current_y - y)
        if lookahead_distance < distance:
            closest_point = (x, y)
            index = i
            break
    
    # 목표점 설정
    if closest_point is not None:
        target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)

        if target_heading < 0:
            target_heading += 2 * math.pi
        else :
            pass

        desired_steering_angle = target_heading - current_heading
    else:
        target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)

        if target_heading < 0:
            target_heading += 2 * math.pi
        else :
            pass

        desired_steering_angle = target_heading - current_heading
        index = len(path) - 1
    
    # 각도 정규화
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    
    # 각도 제한 (30도 이상이면 45도로 제한하고 정지)
    # if desired_steering_angle > math.pi/6 or desired_steering_angle < -math.pi/6:
    #     sign = 1 if desired_steering_angle > 0 else -1
    #     desired_steering_angle = sign * math.pi/4
    #     v = 0.0
    
    return v, float(desired_steering_angle*180/math.pi), index # degree 단위로 변환

class UTMPurePursuit(Node):
    def __init__(self):
        super().__init__('utm_pure_pursuit')
        
        # Pure Pursuit 파라미터
        self.waypoint_tolerance = 0.3  # waypoint 도달 판정 거리 (미터)
        
        # 로봇 상태
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Waypoint 설정 (control.py의 goal과 유사)

        self.waypoints = [
            (329983.719725, 4123210.415129),  # 첫 번째 waypoint
            (329977.396338, 4123210.620894),   # 두 번째 waypoint
            (329964.442297, 4123208.130461),   # 세 번째 waypoint
            (329955.384840, 4123213.997123)   # 네 번째 waypoint
        ]
        
        
        
        # control.py와 동일한 flag 시스템
        self.flag = 0  # 0: 대기, 1: 경로 생성, 2: 추적 중
        self.path = []
        self.i = 0  # 경로 인덱스
        self.current_waypoint_index = 0  # 현재 추적 중인 waypoint 인덱스
        self.first_odometry_received = False  # 첫 번째 odometry 수신 여부
        self.global_path_generated = False  # 전체 경로 생성 완료 여부
        
        # IMU 보정각도 로드
        self.imu_calibration_angle = self.load_imu_calibration_angle()
        
        # ROS2 인터페이스 설정 (control.py와 동일한 구조)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 토픽 구독자
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/global_utm',
            self.info_callback,
            10
        )
        
        # control.py와 동일한 타이머 주기
        timer_period = 0.01  # 100Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info("UTM Pure Pursuit 노드가 시작되었습니다.")
        self.get_logger().info(f"IMU 보정각도: {self.imu_calibration_angle:.6f} rad ({math.degrees(self.imu_calibration_angle):.2f}°)")
        self.get_logger().info(f"총 {len(self.waypoints)}개의 waypoint가 설정되었습니다.")
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f"  Waypoint {i+1}: ({wp[0]:.3f}, {wp[1]:.3f})")
        self.get_logger().info("Odometry 데이터 수신 후 자동으로 추적을 시작합니다...")
    
    def load_imu_calibration_angle(self):
        """IMU 보정각도 파일에서 읽기"""
        import os
        
        # 여러 경로에서 파일 찾기
        possible_paths = [
            "imu_calibration_angle.txt",  # 현재 디렉토리
            "/home/jh/ros2_workspace/src/imu_calibration_angle.txt",  # 절대 경로
            os.path.join(os.path.dirname(__file__), "imu_calibration_angle.txt"),  # 스크립트 디렉토리
            os.path.join(os.getcwd(), "imu_calibration_angle.txt")  # 작업 디렉토리
        ]
        
        for file_path in possible_paths:
            try:
                if os.path.exists(file_path):
                    with open(file_path, 'r') as f:
                        # 첫 번째 줄: 라디안 단위 보정각도
                        calibration_angle_rad = float(f.readline().strip())
                        self.get_logger().info(f"IMU 보정각도 파일 로드 성공: {file_path}")
                        self.get_logger().info(f"보정각도: {calibration_angle_rad:.6f} rad")
                        return calibration_angle_rad
            except Exception as e:
                self.get_logger().debug(f"파일 {file_path} 읽기 실패: {e}")
                continue
        
        # 모든 경로에서 파일을 찾지 못한 경우
        self.get_logger().warn("IMU 보정각도 파일(imu_calibration_angle.txt)을 찾을 수 없습니다.")
        self.get_logger().warn("검색한 경로들:")
        for path in possible_paths:
            self.get_logger().warn(f"  - {path}")
        self.get_logger().warn("기본값 0을 사용합니다.")
        return 0.0
    
    def generate_global_path(self):
        """전체 waypoint를 사용하여 글로벌 경로 생성"""
        if len(self.waypoints) < 2:
            self.get_logger().error("waypoint가 2개 미만입니다. 경로 생성이 불가능합니다.")
            return False
        
        # 현재 위치를 시작점으로 하고 모든 waypoint를 포함한 경로 생성
        path_points = [(self.x, self.y)]  # 현재 위치를 시작점으로
        path_points.extend(self.waypoints)  # 모든 waypoint 추가
        
        self.get_logger().info("🗺️ 전체 경로 생성 중...")
        self.get_logger().info(f"   총 경로점: {len(path_points)}개")
        self.get_logger().info(f"   시작점: ({self.x:.3f}, {self.y:.3f})")
        self.get_logger().info(f"   종료점: ({self.waypoints[-1][0]:.3f}, {self.waypoints[-1][1]:.3f})")
        
        # B-Spline으로 경로 스무딩 (더 많은 점으로 스무딩)
        total_distance = 0.0
        for i in range(len(path_points) - 1):
            dist = math.sqrt((path_points[i+1][0] - path_points[i][0])**2 + 
                           (path_points[i+1][1] - path_points[i][1])**2)
            total_distance += dist
        
        # 거리에 비례하여 스무딩 점 수 결정 (1m당 10개 점)
        smoothing_points = max(100, int(total_distance * 10))
        
        self.path = bspline_planning(path_points, smoothing_points)
        
        self.get_logger().info("✅ 전체 경로 생성 완료!")
        self.get_logger().info(f"   스무딩된 경로점: {len(self.path)}개")
        self.get_logger().info(f"   총 거리: {total_distance:.2f}m")
        self.get_logger().info(f"   스무딩 밀도: {smoothing_points}개 점")
        
        return True
        
    def info_callback(self, msg):
        """Odometry 데이터 처리"""
        # UTM 좌표 추출
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # 쿼터니언에서 yaw 각도 추출
        raw_yaw = euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        
        # IMU 보정각도 적용
        self.yaw = raw_yaw + self.imu_calibration_angle
        
        # 첫 번째 odometry 수신 시 로그 출력 및 경로 생성 시작
        if not self.first_odometry_received:
            self.get_logger().info("=" * 60)
            self.get_logger().info("📍 로봇 초기 위치 정보")
            self.get_logger().info(f"   X 좌표: {self.x:.6f} m")
            self.get_logger().info(f"   Y 좌표: {self.y:.6f} m")
            self.get_logger().info(f"   Raw Yaw: {raw_yaw:.6f} rad ({math.degrees(raw_yaw):.2f}°)")
            self.get_logger().info(f"   보정각도: {self.imu_calibration_angle:.6f} rad ({math.degrees(self.imu_calibration_angle):.2f}°)")
            self.get_logger().info(f"   보정된 Yaw: {self.yaw:.6f} rad ({math.degrees(self.yaw):.2f}°)")
            self.get_logger().info("=" * 60)
            self.first_odometry_received = True
            
            # 전체 경로 생성 시작
            if len(self.waypoints) > 0 and not self.global_path_generated:
                self.get_logger().info("🚀 전체 경로 생성을 시작합니다!")
                if self.generate_global_path():
                    self.global_path_generated = True
                    self.flag = 2  # 바로 추적 시작
                    self.get_logger().info("🎯 경로 추적을 시작합니다!")
                else:
                    self.get_logger().error("경로 생성에 실패했습니다.")
    
    def timer_callback(self):
        """메인 제어 루프 (전체 경로 추적)"""
        if self.flag == 2:
            # Pure Pursuit 제어 실행
            twist = Twist()
            twist.linear.x, twist.angular.z, self.i = pure_pursuit(
                self.x, self.y, self.yaw, self.path, self.i
            )
            
            
            # 경로 완료 판정 (마지막 waypoint 도달)
            if self.i >= len(self.path) - 1:
                # 경로 완료
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.flag = 0
                self.get_logger().info("🎉 전체 경로 완료!")
                self.get_logger().info("✅ 미션 성공!")
                self.get_logger().info("로봇이 정지합니다.")
            
            # 현재 waypoint 도달 판정 (진행 상황 표시용)
            if self.current_waypoint_index < len(self.waypoints):
                current_waypoint = self.waypoints[self.current_waypoint_index]
                distance_to_waypoint = math.sqrt(
                    (self.x - current_waypoint[0])**2 + (self.y - current_waypoint[1])**2
                )
                
                if distance_to_waypoint < self.waypoint_tolerance:  # 1m 이내면 도달로 판정
                    waypoint_num = self.current_waypoint_index + 1
                    self.get_logger().info("=" * 50)
                    self.get_logger().info(f"🎯 Waypoint {waypoint_num} 도달!")
                    self.get_logger().info(f"   위치: ({current_waypoint[0]:.3f}, {current_waypoint[1]:.3f})")
                    self.get_logger().info(f"   거리: {distance_to_waypoint:.2f}m")
                    self.get_logger().info(f"   진행률: {waypoint_num}/{len(self.waypoints)}")
                    self.get_logger().info("=" * 50)
                    
                    self.current_waypoint_index += 1
            
            self.publisher.publish(twist)
            
    
    


def main(args=None):
    rclpy.init(args=args)
    
    # UTM Pure Pursuit 노드 생성
    utm_pure_pursuit = UTMPurePursuit()
    
    try:
        rclpy.spin(utm_pure_pursuit)
    except KeyboardInterrupt:
        utm_pure_pursuit.get_logger().info("노드 종료 중...")
    finally:
        utm_pure_pursuit.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()