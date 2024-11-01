import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
import subprocess
import yaml
from my_msgs.action import StopFlag  # Actionメッセージのインポート
import std_msgs.msg as std_msgs
import nav_msgs.msg as nav_msgs
import sensor_msgs.msg as sensor_msgs
import numpy as np
import math
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import yaml
import os
import time
import geometry_msgs.msg as geometry_msgs
import glob
import cv2
from std_msgs.msg import Int8MultiArray
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import transforms3d
from geometry_msgs.msg import Quaternion

class TrafficLightWaypointMonitor(Node):
    def __init__(self):
        super().__init__('trafficlight_waypoint_monitor')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom_wheel',
            self.odom_callback,
            10)
        # Waypoint YAMLファイルを読み込む
        folder_path = os.path.expanduser('~/ros2_ws/src/nakaniwa')
        # フォルダ内のpgmファイルを検索
        pgm_files = glob.glob(os.path.join(folder_path, '*.pgm'))
        pgm_file_count = len(pgm_files)
        #Global reflect map load: PGM、JPEG、YAMLファイルを行列形式で読み込み
        map_base_name = "nakaniwa_test" # 対象画像の名前
        folder_path = os.path.expanduser('~/ros2_ws/src/nakaniwa')
        map_file_path = os.path.join(folder_path, map_base_name) # パスの連結
        
        global_reflect_map_pgms = []
        map_resolution = []; map_origin=[]; map_occupied_thresh=[];map_free_thresh=[];
        reflect_map_obs_matrices = []
        for map_number in range(pgm_file_count):
            map_number_str = str(map_number).zfill(3)
            #load pgm
            pgm_filename = os.path.join(folder_path, f'{map_file_path}_{map_number_str}' + ".pgm")
            print(f"pgm_filename: {pgm_filename}")
            global_reflect_map_pgm = cv2.imread(pgm_filename, cv2.IMREAD_GRAYSCALE)
            global_reflect_map_pgms.append(global_reflect_map_pgm)
            #load yaml
            yaml_filename = os.path.join(folder_path, f'{map_file_path}_{map_number_str}' + ".yaml")
            with open(yaml_filename, 'r') as yaml_file:
                map_yaml_data = yaml.safe_load(yaml_file)
            map_resolution.append(map_yaml_data['resolution'])
            map_origin.append(map_yaml_data['origin'])
            map_occupied_thresh.append(map_yaml_data['occupied_thresh'])
            map_free_thresh.append(map_yaml_data['free_thresh'])
            #load jpeg
            jpeg_filename = os.path.join(folder_path, f'{map_file_path}_{map_number_str}' + ".jpeg")
            reflect_map_obs = cv2.imread(jpeg_filename)
            red_judge1 = reflect_map_obs[:,:,0] < 100
            red_judge2 = reflect_map_obs[:,:,2] > 200
            reflect_map_obs_data = red_judge1 * red_judge2 * 100
            reflect_map_obs_matrices.append(reflect_map_obs_data)
            
        self.global_reflect_map_pgm = np.stack(global_reflect_map_pgms)
        self.global_reflect_map_resolution = np.stack(map_resolution)
        self.global_reflect_map_origin = np.stack(map_origin)
        self.global_reflect_map_occupied_thresh = np.stack(map_occupied_thresh)
        self.global_reflect_map_free_thresh = np.stack(map_free_thresh)
        self.reflect_map_obs_matrices = np.stack(reflect_map_obs_matrices)
        
        #print(f"self.global_reflect_map_pgm: {self.global_reflect_map_pgm.shape}")
        #print(f"self.global_reflect_map_pgm: {self.global_reflect_map_pgm.shape[0]}")
        #print(f"self.global_reflect_map_pgm[0]: {self.global_reflect_map_pgm[0]}")
        #print(f"self.global_reflect_map_resolution[0]: {self.global_reflect_map_resolution[0]}")
        #print(f"self.global_reflect_map_origin[0]: {self.global_reflect_map_origin[0]}")
        #print(f"self.reflect_map_obs_matrices[0]: {self.reflect_map_obs_matrices[0]}")
        
        wp_x = []; wp_y=[]; wp_z=[];
        for wp_number in range(self.global_reflect_map_pgm.shape[0]-1):
            x_offset = (len(self.global_reflect_map_pgm[wp_number][0]) * self.global_reflect_map_resolution[0])/2
            y_offset = (len(self.global_reflect_map_pgm[wp_number][1]) * self.global_reflect_map_resolution[0])/2
            x = self.global_reflect_map_origin[wp_number][0] + x_offset #pgm end + (pgm len * grid)/2
            y = self.global_reflect_map_origin[wp_number][1] + y_offset #pgm end + (pgm len * grid)/2
            z = self.global_reflect_map_origin[wp_number][2]
            wp_x.append(x)
            wp_y.append(y)
            wp_z.append(z)
        self.waypoints = np.array([wp_x,wp_y,wp_z])
        print(f"self.waypoints ={self.waypoints}")
        print(f"self.waypoints ={self.waypoints.shape}")
        self.waypoint = list(map(tuple, self.waypoints[:2].T))
        print(f"self.waypoint ={self.waypoint}")
        print(len(self.waypoint))
        #############################
        
        # self.waypoints = self.load_waypoints('/home/ubuntu/ros2_ws/src/kbkn_maps/waypoints/gazebo/orange_hosei/slam_toolbox/waypoints.yaml')
        self.current_waypoint_index = 0
        self.waypoints_to_trigger_action = [0]  # 特定のウェイポイントでアクションを送信
        self.action_client = ActionClient(self, StopFlag, 'traffic_flag')  # ActionClientの設定
        self.action_sent = False  # アクションが送信されたかを追跡
        self.traffic_action = False # traffic_actionするかの変数
    
    '''
    def load_waypoints(self, filepath):
        with open(filepath, 'r') as file:
            data = yaml.safe_load(file)
        waypoints = [(point['point']['x'], point['point']['y']) for point in data['waypoints']]
        return waypoints
    '''
    
    def odom_callback(self, msg):
        current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        # print(f"current_position ={self.current_position}")
        if self.current_waypoint_index < len(self.waypoint):
            target_position = self.waypoint[self.current_waypoint_index]
            if self.is_at_waypoint(current_position, target_position):
                self.get_logger().info(f'Waypoint {self.current_waypoint_index} reached!')

                # 特定のインデックスのウェイポイントに到達した場合にアクションを送信
                if self.current_waypoint_index in self.waypoints_to_trigger_action and not self.action_sent:
                    self.traffic_action = True
                    self.send_action_request()
                    self.action_sent = True  # アクションが送信されたことをマーク

                self.current_waypoint_index += 1

    def is_at_waypoint(self, current_position, target_position):
        return (abs(current_position[0] - target_position[0]) < 5.0 and
                abs(current_position[1] - target_position[1]) < 10.0)

    # サーバーにアクションを送信する関数
    def send_action_request(self):
        goal_msg = StopFlag.Goal()
        
        # traffic_action変数の状態でaの値を決定
        if self.traffic_action:
            goal_msg.a = 1  # start judge
        else:
            goal_msg.a = 0  # 
            
        goal_msg.b = 2  # 任意の値を設定

        # アクションサーバーが利用可能になるまで待機
        self.action_client.wait_for_server()

        # アクションを非同期で送信
        self.future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.response_callback)

    # フィードバックを受け取るコールバック関数
    def feedback_callback(self, feedback):
        self.get_logger().info(f"Received feedback: {feedback.feedback.rate}")

    # 結果を受け取るコールバック関数
    def response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    # 結果のコールバック
    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.sum}")

def main(args=None):
    rclpy.init(args=args)
    trafficlight_waypoint_monitor = TrafficLightWaypointMonitor()
    # ノード終了の待機
    rclpy.spin(trafficlight_waypoint_monitor)


if __name__ == '__main__':
    main()

