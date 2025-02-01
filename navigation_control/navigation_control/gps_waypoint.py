# rclpy (ROS 2のpythonクライアント)の機能を使えるようにします。
import rclpy
# rclpy (ROS 2のpythonクライアント)の機能のうちNodeを簡単に使えるようにします。こう書いていない場合、Nodeではなくrclpy.node.Nodeと書く必要があります。
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import numpy as np
import time
import threading
from geopy.distance import geodesic  
import tkinter as tk
import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs
import math
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
import yaml
import os

class GPSAverageNode(Node):
    def __init__(self):
        super().__init__('gps_average_node')
        self.subscription = self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)

        self.data = []
        self.start_time = None
        self.is_collecting = False
        self.waypoints = None  

        self.ref_points = [
            (35.43, 139.32),
            (35.44, 139.33),
            (35.42, 139.31),
            (35.45, 139.34)
        ]

        # Tkinter UI
        self.root = tk.Tk()
        self.root.title("GPS Data Collection")
        self.button = tk.Button(self.root, text="Start Collection", command=self.start_collection)
        self.button.pack()

    def rotate_coordinates(self, x_diff, y_diff, vehicle_angle):
        """ 指定した角度 (度単位) で (x, y) を回転させる """
        angle_rad = math.radians(vehicle_angle)  # 角度をラジアンに変換
        x_rot = x_diff * math.cos(angle_rad) - y_diff * math.sin(angle_rad)
        y_rot = x_diff * math.sin(angle_rad) + y_diff * math.cos(angle_rad)
        return x_rot, y_rot

    def start_collection(self):
        if not self.is_collecting:
            self.get_logger().info("ボタンが押されました。GPSデータ収集開始。")
            self.data = []
            self.start_time = time.time()
            self.is_collecting = True

    def gps_callback(self, msg):
        if self.is_collecting:
            elapsed_time = time.time() - self.start_time
            if elapsed_time <= 10.0:
                self.data.append((msg.latitude, msg.longitude))
                self.get_logger().info(f'受信: 緯度={msg.latitude}, 経度={msg.longitude}')
            else:
                self.calculate_average()

    def calculate_average(self):
        if self.data:
            avg_lat = sum(lat for lat, lon in self.data) / len(self.data)
            avg_lon = sum(lon for lat, lon in self.data) / len(self.data)

            self.get_logger().info(f'10秒間の平均値: 緯度={avg_lat}, 経度={avg_lon}')
            self.get_logger().info('基準座標との差を x-y 座標に変換:')

            # 手動で設定する車両の向き (例: 30°)
            vehicle_angle = 30  # ここを手動で変更

            waypoints_list = []
            for i, (ref_lat, ref_lon) in enumerate(self.ref_points):
                y_diff = geodesic((ref_lat, avg_lon), (avg_lat, avg_lon)).meters
                if ref_lat < avg_lat:
                    y_diff *= -1  

                x_diff = geodesic((avg_lat, ref_lon), (avg_lat, avg_lon)).meters
                if ref_lon < avg_lon:
                    x_diff *= -1  

                # 車両の向きを考慮して回転
                x_rot, y_rot = self.rotate_coordinates(x_diff, y_diff, vehicle_angle)

                waypoints_list.append([x_rot, y_rot, 0.0])  

            self.waypoints = np.array(waypoints_list).T  # 形状を [3, N] に変換
            self.get_logger().info(f'waypoints 配列を保存しました: {self.waypoints}')

            self.is_collecting = False  

            # 計算が終わったら WaypointManager を起動
            self.launch_waypoint_manager()

    def launch_waypoint_manager(self):
        self.get_logger().info("WaypointManager を起動します。")
        waypoint_manager = WaypointManager(self.waypoints)
        rclpy.spin(waypoint_manager)
        waypoint_manager.destroy_node()

    def run(self):
        self.root.mainloop()

# C++と同じく、Node型を継承します。
class WaypointManager(Node):
    # コンストラクタです、PcdRotationクラスのインスタンスを作成する際に呼び出されます。
    def __init__(self, waypoints):
        # 継承元のクラスを初期化します。
        super().__init__('waypoint_manager_node')
        
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth = 10
        )
        
        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth = 10
        )
        
        # Subscriptionを作成。
        self.subscription = self.create_subscription(nav_msgs.Odometry,'/odom_wheel', self.get_odom, qos_profile_sub)
        self.subscription  # 警告を回避するために設置されているだけです。削除しても挙動はかわりません。
        
        # タイマーを0.1秒（100ミリ秒）ごとに呼び出す
        self.timer = self.create_timer(0.1, self.waypoint_manager)
        
        # Publisherを作成
        self.current_waypoint_publisher = self.create_publisher(geometry_msgs.PoseArray, 'current_waypoint', qos_profile) #set publish pcd topic name
        
        #パラメータ
        #waypoint init
        self.current_waypoint = 0
        self.stop_flag = 0
        
        #positon init
        self.position_x = 0.0 #[m]
        self.position_y = 0.0 #[m]
        self.position_z = 0.0 #[m]
        self.theta_x = 0.0 #[deg]
        self.theta_y = 0.0 #[deg]
        self.theta_z = 0.0 #[deg]
        
        self.waypoints = waypoints
        print(f"self.waypoints ={self.waypoints}")
        
    def waypoint_manager(self):
        self.time_stamp = self.get_clock().now().to_msg()
        #self.get_logger().info('waypoint manager cntl')
        position_x=self.position_x; position_y=self.position_y; 
        theta_x=self.theta_x; theta_y=self.theta_y; theta_z=self.theta_z;
        
        #waypoint theta & dist
        relative_point_x = self.waypoints[0,self.current_waypoint] - position_x
        relative_point_y = self.waypoints[1,self.current_waypoint] - position_y
        relative_point = np.vstack((relative_point_x, relative_point_y, self.waypoints[2,self.current_waypoint]))
        relative_point_rot, t_point_rot_matrix = rotation_xyz(relative_point, theta_x, theta_y, -theta_z)
        waypoint_rad = math.atan2(relative_point_rot[1], relative_point_rot[0])
        waypoint_dist = math.sqrt(relative_point_x**2 + relative_point_y**2)
        waypoint_theta = abs(waypoint_rad * (180 / math.pi))
        
        #set judge dist
        if abs(waypoint_theta) > 90:
            determine_dist = 8
        else:
            determine_dist = 3
        #check if the waypoint reached
        if waypoint_dist < determine_dist:
            self.current_waypoint += 1
            if self.current_waypoint > (len(self.waypoints[0,:]) - 1):
                self.stop_flag = 1
                self.get_logger().info('GOAL : stop_flag = %f' % (stop_flag))
        self.get_logger().info('current_waypoint:x = %f, y = %f : waypoint_no = %f' % (self.waypoints[0,self.current_waypoint], self.waypoints[1,self.current_waypoint], self.current_waypoint))
        
        #publish
        pose_array = self.current_waypoint_msg(self.waypoints[:,self.current_waypoint], 'map')
        self.current_waypoint_publisher.publish(pose_array)
        
    def get_odom(self, msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        self.position_z = msg.pose.pose.position.z
        
        flio_q_x = msg.pose.pose.orientation.x
        flio_q_y = msg.pose.pose.orientation.y
        flio_q_z = msg.pose.pose.orientation.z
        flio_q_w = msg.pose.pose.orientation.w
        
        roll, pitch, yaw = quaternion_to_euler(flio_q_x, flio_q_y, flio_q_z, flio_q_w)
        
        self.theta_x = 0 #roll /math.pi*180
        self.theta_y = 0 #pitch /math.pi*180
        self.theta_z = yaw /math.pi*180
        
    def current_waypoint_msg(self, waypoint, set_frame_id):
        pose_array = geometry_msgs.PoseArray()
        pose_array.header.frame_id = set_frame_id
        pose_array.header.stamp = self.time_stamp
        pose = geometry_msgs.Pose()
        pose.position.x = waypoint[0]
        pose.position.y = waypoint[1]
        pose.position.z = waypoint[2]
        
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        pose_array.poses.append(pose)
        
        return pose_array

def rotation_xyz(pointcloud, theta_x, theta_y, theta_z):
    theta_x = math.radians(theta_x)
    theta_y = math.radians(theta_y)
    theta_z = math.radians(theta_z)
    rot_x = np.array([[ 1,                 0,                  0],
                      [ 0, math.cos(theta_x), -math.sin(theta_x)],
                      [ 0, math.sin(theta_x),  math.cos(theta_x)]])
    
    rot_y = np.array([[ math.cos(theta_y), 0,  math.sin(theta_y)],
                      [                 0, 1,                  0],
                      [-math.sin(theta_y), 0, math.cos(theta_y)]])
    
    rot_z = np.array([[ math.cos(theta_z), -math.sin(theta_z), 0],
                      [ math.sin(theta_z),  math.cos(theta_z), 0],
                      [                 0,                  0, 1]])
    rot_matrix = rot_z.dot(rot_y.dot(rot_x))
    #print(f"rot_matrix ={rot_matrix}")
    #print(f"pointcloud ={pointcloud.shape}")
    rot_pointcloud = rot_matrix.dot(pointcloud)
    return rot_pointcloud, rot_matrix
    
def quaternion_to_euler(x, y, z, w):
    # クォータニオンから回転行列を計算
    rot_matrix = np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x**2 + z**2), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x**2 + y**2)]
    ])

    # 回転行列からオイラー角を抽出
    roll = np.arctan2(rot_matrix[2, 1], rot_matrix[2, 2])
    pitch = np.arctan2(-rot_matrix[2, 0], np.sqrt(rot_matrix[2, 1]**2 + rot_matrix[2, 2]**2))
    yaw = np.arctan2(rot_matrix[1, 0], rot_matrix[0, 0])
    return roll, pitch, yaw
        

def main(args=None):
    rclpy.init(args=args)
    node = GPSAverageNode()
    rclpy_thread = threading.Thread(target=rclpy.spin, args=(node,))
    rclpy_thread.start()
    node.run()  
    node.destroy_node()
    rclpy.shutdown()
    rclpy_thread.join()

if __name__ == '__main__':
    main()

