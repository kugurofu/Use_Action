import rclpy
import time
from rclpy.action import ActionServer
from rclpy.node import Node
from my_msgs.action import StopFlag

# サーバー
class MyServer(Node):
    # 初期化
    def __init__(self):
        super().__init__('my_server')

        # サーバーの生成
        self.server = ActionServer(self,
            StopFlag, "stop_flag", self.listener_callback)
        
        # 変数の追加
        self.stop_flag = 0

    # リクエストの受信時に呼ばれる
    def listener_callback(self, goal_handle):
        self.get_logger().info(f"Received goal with a: {goal_handle.request.a}, b: {goal_handle.request.b}")
        
        # クライアントから送られたaをstop_flagに代入
        self.stop_flag = goal_handle.request.a
        print(f"stop_flag set to: {self.stop_flag}")
        
        
        # フィードバックの返信
        for i in range(10):
            feedback = StopFlag.Feedback()
            feedback.rate = i * 0.1
            goal_handle.publish_feedback(feedback)
            time.sleep(0.5)

        # レスポンスの返信
        goal_handle.succeed()
        result = StopFlag.Result()
        result.sum = goal_handle.request.a + goal_handle.request.b  # 結果の計算
        return result

# メイン
def main(args=None):
    # ROS通信の初期化
    rclpy.init(args=args)

    # サーバーの生成
    server = MyServer()

    # ノード終了の待機
    rclpy.spin(server)

if __name__ == '__main__':
    main()
