import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray  # PoseArray メッセージをインポート
import yaml
from my_msgs.action import StopFlag  # アクションメッセージのインポート
import tkinter as tk
import threading

class WaypointMonitor(Node):
    def __init__(self):
        super().__init__('waypoint_monitor')
        
        # Odometry のサブスクリプション
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # 現在のウェイポイントのサブスクリプション
        self.waypoint_subscription = self.create_subscription(
            PoseArray,
            '/current_waypoint',
            self.current_waypoint_callback,
            10)
        
        # ウェイポイントをファイルから読み込む
        # self.waypoints = self.load_waypoints('/home/ubuntu/ros2_ws/src/kbkn_maps/waypoints/gazebo/orange_hosei/slam_toolbox/waypoints.yaml')
        self.current_waypoint_index = 0
        self.waypoints_to_trigger_action = [0]  # アクションをトリガーする特定のウェイポイントインデックス
        self.action_client = ActionClient(self, StopFlag, 'stop_flag')  # ActionClient の設定
        self.action_sent = False  # アクションが送信されたかを追跡
        self.stop = False  # 停止制御用の変数
        
        # Tkinter のウィンドウとボタンの設定
        self.root = tk.Tk()
        self.root.title("Waypoint Monitor Control")
        self.button = tk.Button(self.root, text="Resume", command=self.resume_action)
        self.button.pack()

    def load_waypoints(self, filepath):
        with open(filepath, 'r') as file:
            data = yaml.safe_load(file)
        waypoints = [(point['point']['x'], point['point']['y']) for point in data['waypoints']]
        return waypoints

    def current_waypoint_callback(self, msg):
        # 受信した PoseArray に基づいて現在のウェイポイントインデックスを更新
        if msg.poses:
            # 受信したポーズの数とウェイポイントの長さに基づいてインデックスを設定
            self.current_waypoint_index = min(len(msg.poses) - 1, len(self.waypoints) - 1)
            self.get_logger().info(f"現在のウェイポイントインデックスが更新されました: {self.current_waypoint_index}")

    def odom_callback(self, msg):
        current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.current_waypoint_index < len(self.waypoints):
            target_position = self.waypoints[self.current_waypoint_index]
            if self.is_at_waypoint(current_position, target_position):
                self.get_logger().info(f'ウェイポイント {self.current_waypoint_index} に到達しました！')

                # 特定のウェイポイントに到達した場合にアクションを送信
                if self.current_waypoint_index in self.waypoints_to_trigger_action and not self.action_sent:
                    self.stop = True
                    self.send_action_request()
                    self.action_sent = True  # アクションが送信されたことをマーク

                self.current_waypoint_index += 1

    def is_at_waypoint(self, current_position, target_position):
        return (abs(current_position[0] - target_position[0]) < 0.3 and
                abs(current_position[1] - target_position[1]) < 0.3)

    # サーバーにアクションを送信する関数
    def send_action_request(self):
        goal_msg = StopFlag.Goal()
        
        # 停止変数に基づいて 'a' の値を決定
        goal_msg.a = 1 if self.stop else 0  # 停止するか続行するか
        goal_msg.b = 2  # 任意の値を設定

        # アクションサーバーが利用可能になるまで待機
        self.action_client.wait_for_server()

        # アクションを非同期で送信
        self.future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.response_callback)

    # フィードバックを受け取るコールバック関数
    def feedback_callback(self, feedback):
        self.get_logger().info(f"フィードバックを受信しました: {feedback.feedback.rate}")

    # 結果を受け取るコールバック関数
    def response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("目標が拒否されました")
            return

        self.get_logger().info("目標が受け入れられました")

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    # 結果のコールバック
    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"結果: {result.sum}")

    def resume_action(self):
        # ボタンを押したときに stop を False にしてアクションを再送信
        self.stop = True
        self.get_logger().info("停止フラグをリセットしました")
        self.send_action_request()

    def run(self):
        # Tkinter のメインループを開始
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    waypoint_monitor = WaypointMonitor()
    # rclpy スピンを別スレッドで実行
    rclpy_thread = threading.Thread(target=rclpy.spin, args=(waypoint_monitor,))
    rclpy_thread.start()
    # Tkinter GUI のループを実行
    waypoint_monitor.run()
    # スレッド終了後にノードを破棄
    waypoint_monitor.destroy_node()
    rclpy.shutdown()
    rclpy_thread.join()

if __name__ == '__main__':
    main()

