import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation as R

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        # 상태 벡터: [x, y, vx, vy]
        self.x = np.zeros(4)
        self.P = np.eye(4)  # 오차 공분산 행렬
        self.F = np.eye(4)  # 상태 전이 행렬
        self.Q = np.eye(4) * 0.1  # 프로세스 잡음

        # 관측 행렬: x와 y만 측정
        self.H = np.zeros((2, 4))
        self.H[:2, :2] = np.eye(2)

        # 측정 잡음 공분산: x와 y만 고려
        self.R = np.eye(2) * 0.001

        self.last_pose = None
        self.last_time = None

        # ROS2 구독 및 게시
        self.pose_sub = self.create_subscription(
            PoseStamped, '/hmcar1/tracked_pose', self.pose_callback, 10
        )
        self.odom_pub = self.create_publisher(Odometry, '/kalman_odom', 10)

    def pose_callback(self, msg):
        # 측정값: x와 y만 반영
        z = np.array([msg.pose.position.x, msg.pose.position.y])

        # Delta time 계산
        dt = self.get_delta_time(msg)
        if dt is None:
            return  # 첫 데이터 수신 시 건너뜀

        # 상태 전이 행렬 업데이트
        self.update_state_transition(dt)

        # Prediction 단계
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

        # Update 단계
        y = z - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x += np.dot(K, y)
        self.P = np.dot(np.eye(4) - np.dot(K, self.H), self.P)

        # Odometry 메시지 생성 및 게시
        self.publish_odometry(msg)

    def update_state_transition(self, dt):
        # 위치와 속도의 관계를 상태 전이 행렬에 반영
        self.F[:2, 2:] = np.eye(2) * dt

    def get_delta_time(self, msg):
        # 이전 위치 및 시간 정보를 기반으로 dt 계산
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            self.last_pose = np.array([msg.pose.position.x, msg.pose.position.y])
            return None  # 첫 데이터는 건너뜀

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        return dt

    def publish_odometry(self, msg):
        # Odometry 메시지 작성
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map_hmcar1"

        # 필터링된 위치
        odom_msg.pose.pose.position.x = self.x[0]
        odom_msg.pose.pose.position.y = self.x[1]
        odom_msg.pose.pose.position.z = msg.pose.position.z  # z 값 그대로 사용

        # Orientation 그대로 복사
        odom_msg.pose.pose.orientation = msg.pose.orientation

        # 필터링된 속도
        odom_msg.twist.twist.linear.x = self.x[2]
        odom_msg.twist.twist.linear.y = self.x[3]
        odom_msg.twist.twist.linear.z = 0.0  # z 축 속도는 사용하지 않음

        # 공분산: 위치
        pose_covariance = np.zeros(36)
        pose_covariance[0] = self.P[0, 0]  # x-x 공분산
        pose_covariance[1] = self.P[0, 1]  # x-y 공분산
        pose_covariance[6] = self.P[1, 0]  # y-x 공분산
        pose_covariance[7] = self.P[1, 1]  # y-y 공분산
        pose_covariance[35] = 0.1  # z 방향은 임의값
        odom_msg.pose.covariance = pose_covariance

        # 공분산: 속도
        twist_covariance = np.zeros(36)
        twist_covariance[0] = self.P[2, 2]  # vx-vx 공분산
        twist_covariance[1] = self.P[2, 3]  # vx-vy 공분산
        twist_covariance[6] = self.P[3, 2]  # vy-vx 공분산
        twist_covariance[7] = self.P[3, 3]  # vy-vy 공분산
        odom_msg.twist.covariance = twist_covariance

        # 메시지 게시
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
