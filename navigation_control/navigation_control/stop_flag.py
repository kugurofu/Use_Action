import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from my_msgs.action import StopFlag  # アクションのメッセージ型
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class TrafficLightMonitor(Node):
    def __init__(self):
        super().__init__('traffic_light_monitor')

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10
        )

        # /traffic_light_statusのサブスクリプション
        self.subscription = self.create_subscription(
            String,
            '/traffic_light_status',
            self.traffic_light_callback,
            qos_profile
        )
        self.subscription  # 無警告のため

        # StopFlagのアクションクライアント
        self._action_client = ActionClient(self, StopFlag, 'set_stop_flag')

    def traffic_light_callback(self, msg):
        self.get_logger().info('Received traffic light status: "%s"' % msg.data)

        if msg.data == 'red':
            self.send_stop_flag(True)  # 赤信号でstop_flag=1を送る
        else:
            self.send_stop_flag(False)  # 赤以外ならstop_flag=0を送る

    def send_stop_flag(self, flag_value):
        goal_msg = StopFlag.Goal()
        goal_msg.data = flag_value

        self.get_logger().info('Sending stop flag: %s' % ('1' if flag_value else '0'))
        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    traffic_light_monitor = TrafficLightMonitor()
    rclpy.spin(traffic_light_monitor)
    traffic_light_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

