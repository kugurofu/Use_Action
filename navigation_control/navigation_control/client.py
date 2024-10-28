import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
import subprocess
import yaml
from my_msgs.action import StopFlag  # Actionメッセージのインポート
import tkinter as tk
import threading

class WaypointMonitor(Node):
    def __init__(self):
        super().__init__('waypoint_monitor')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.waypoints = self.load_waypoints('/home/ubuntu/ros2_ws/src/kbkn_maps/waypoints/gazebo/orange_hosei/slam_toolbox/waypoints.yaml')
        self.current_waypoint_index = 0
        self.waypoints_to_trigger_action = [0]  # 特定のウェイポイントでアクションを送信
        self.action_client = ActionClient(self, StopFlag, 'stop_flag')  # ActionClientの設定
        self.action_sent = False  # アクションが送信されたかを追跡
        self.stop = False # stopするかの変数
        
        # Tkinterウィンドウとボタンの設定
        self.root = tk.Tk()
        self.root.title("Waypoint Monitor Control")
        self.button = tk.Button(self.root, text="Resume", command=self.resume_action)
        self.button.pack()

    def load_waypoints(self, filepath):
        with open(filepath, 'r') as file:
            data = yaml.safe_load(file)
        waypoints = [(point['point']['x'], point['point']['y']) for point in data['waypoints']]
        self.get_logger().info(f' {waypoints} !')
        return waypoints

    def odom_callback(self, msg):
        current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.current_waypoint_index < len(self.waypoints):
            target_position = self.waypoints[self.current_waypoint_index]
            if self.is_at_waypoint(current_position, target_position):
                self.get_logger().info(f'Waypoint {self.current_waypoint_index} reached!')

                # 特定のインデックスのウェイポイントに到達した場合にアクションを送信
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
        
        # stop変数の状態でaの値を決定
        if self.stop:
            goal_msg.a = 1  # stop
        else:
            goal_msg.a = 0  # go
            
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

    def resume_action(self):
        # ボタンを押したときにstopをFalseにしてアクションを再送信
        self.stop = True
        self.get_logger().info("Stop flag reset to False")
        self.send_action_request()

    def run(self):
        # Tkinterのメインループを開始
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    waypoint_monitor = WaypointMonitor()
    # rclpyスピンを別スレッドで実行
    rclpy_thread = threading.Thread(target=rclpy.spin, args=(waypoint_monitor,))
    rclpy_thread.start()
    # Tkinter GUIのループを実行
    waypoint_monitor.run()
    # スレッド終了後にノードを破棄
    waypoint_monitor.destroy_node()
    rclpy.shutdown()
    rclpy_thread.join()

if __name__ == '__main__':
    main()

