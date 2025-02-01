import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import tkinter as tk
import threading
import time
from geopy.distance import geodesic  # 距離計算用ライブラリ

class GPSAverageNode(Node):
    def __init__(self):
        super().__init__('gps_average_node')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )

        self.data = []
        self.start_time = None
        self.is_collecting = False

        # 手動設定する4つの基準座標 (緯度, 経度)
        self.ref_points = [
            (35.43, 139.32), # number 1
            (35.44, 139.33), # number 2
            (35.42, 139.31), # number 3
            (35.45, 139.34)  # number 4
        ]

        # Tkinterウィンドウとボタンの設定
        self.root = tk.Tk()
        self.root.title("GPS Data Collection")
        self.button = tk.Button(self.root, text="Start Collection", command=self.start_collection)
        self.button.pack()

    # ボタンが押されたときにデータ収集を開始
    def start_collection(self):
        if not self.is_collecting:  # すでに開始していない場合のみ実行
            self.get_logger().info("ボタンが押されました。GPSデータ収集開始。")
            self.data = []
            self.start_time = time.time()
            self.is_collecting = True

    # `/fix` トピックのデータを受信
    def gps_callback(self, msg):
        if self.is_collecting:
            elapsed_time = time.time() - self.start_time
            if elapsed_time <= 10.0:
                self.data.append((msg.latitude, msg.longitude))
                self.get_logger().info(f'受信: 緯度={msg.latitude}, 経度={msg.longitude}')
            else:
                self.calculate_average()

    # 10秒間の平均を計算し、各基準座標との差分を x-y 座標に変換
    def calculate_average(self):
        if self.data:
            avg_lat = sum(lat for lat, lon in self.data) / len(self.data)
            avg_lon = sum(lon for lat, lon in self.data) / len(self.data)

            self.get_logger().info(f'10秒間の平均値: 緯度={avg_lat}, 経度={avg_lon}')
            self.get_logger().info('基準座標との差を x-y 座標に変換:')

            for i, (ref_lat, ref_lon) in enumerate(self.ref_points):
                # 緯度方向の距離 (m)
                y_diff = geodesic((ref_lat, avg_lon), (avg_lat, avg_lon)).meters
                if ref_lat < avg_lat:
                    y_diff *= -1  # 南側なら負の値に

                # 経度方向の距離 (m)
                x_diff = geodesic((avg_lat, ref_lon), (avg_lat, avg_lon)).meters
                if ref_lon < avg_lon:
                    x_diff *= -1  # 西側なら負の値に

                self.get_logger().info(f'基準座標 {i+1} ({ref_lat}, {ref_lon}) → x={x_diff:.2f}m, y={y_diff:.2f}m')
        else:
            self.get_logger().info('データが収集されませんでした。')

        self.is_collecting = False  # 収集完了後、リセット

    def run(self):
        # Tkinterのメインループを開始
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = GPSAverageNode()
    # rclpyスピンを別スレッドで実行
    rclpy_thread = threading.Thread(target=rclpy.spin, args=(node,))
    rclpy_thread.start()
    # Tkinter GUIのループを実行
    node.run()
    # スレッド終了後にノードを破棄
    node.destroy_node()
    rclpy.shutdown()
    rclpy_thread.join()

if __name__ == '__main__':
    main()

