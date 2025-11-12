#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from pymavlink import mavutil
import time
import threading
from rclpy.qos import qos_profile_sensor_data  # Sensor verileri için uygun QoS profili
import math

class LidarMavlinkBridge(Node):
    def __init__(self):
        super().__init__('lidar_mavlink_bridge')
        # MAVLink bağlantısı: UDP üzerinden yayın yapıyoruz.
        self.mav_connection = mavutil.mavlink_connection('udpout:127.0.0.1:14550',source_system=42)
        
        # /scan topic'ini dinlerken sensor data QoS profilini kullanıyoruz.
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info("Lidar-MAVLink Bridge başlatıldı.")
        
        # Heartbeat mesajlarını periyodik olarak göndermek için bir thread başlatıyoruz.
        self.heartbeat_thread = threading.Thread(target=self.send_heartbeat, daemon=True)
        self.heartbeat_thread.start()

        # Mesaj gönderim frekansını throttle etmek için en son gönderim zamanını tutuyoruz.
        self.last_publish_time = 0.0  # saniye cinsinden
        self.publish_interval = 0.1   # örneğin, 10 Hz: 0.1 saniyede bir gönder

        # Lidarın ön kısmı için açı aralığı (radyan cinsinden)
        self.front_angle_min = -0.5236  # -30° 
        self.front_angle_max = 0.5236   # +30°

    def send_heartbeat(self):
        # Bu metot, 1 saniyede bir HEARTBEAT mesajı gönderir.
        while rclpy.ok():
            self.mav_connection.mav.heartbeat_send(
                1,  # MAV_TYPE: örneğin 1 = Generic Copter
                6,  # MAV_AUTOPILOT: örneğin 6 = Generic Autopilot
                0,  # base_mode
                0,  # custom_mode
                0   # system_status
            )
            time.sleep(1)  # 1 saniyede bir gönder

    def scan_callback(self, scan_msg: LaserScan):
        current_time = time.time()
        # Eğer belirlenen yayın aralığı henüz geçmediyse mesaj gönderimini atla.
        if current_time - self.last_publish_time < self.publish_interval:
            return
        self.last_publish_time = current_time

        # LaserScan mesajının açısal bilgilerini kullanarak,
        # sadece ön kısmı (-30° ile +30°) kapsayan beam'leri seçelim.
        # angle_min: İlk beam'in açısı, angle_increment: her beam arasındaki artış.
        try:
            # Hesaplama: istenen aralığın indekslerini belirleyelim.
            first_index = int((self.front_angle_min - scan_msg.angle_min) / scan_msg.angle_increment)
            last_index = int((self.front_angle_max - scan_msg.angle_min) / scan_msg.angle_increment)
        except Exception as e:
            self.get_logger().error(f"Açı hesaplamasında hata: {e}")
            return

        # İndeksleri sınırlandırma
        first_index = max(0, first_index)
        last_index = min(len(scan_msg.ranges)-1, last_index)

        # Seçilen aralıktaki verileri alalım
        front_ranges = list(scan_msg.ranges[first_index:last_index+1])
        
        # Geçerli (0.1 m'den büyük) değerleri filtreleyelim.
        valid_values = [r for r in front_ranges if r > 0.1]
        if valid_values:
            avg_distance = sum(valid_values) / len(valid_values)
        else:
            avg_distance = 0.0

        self.get_logger().info(f"Front beam range indices: {first_index} to {last_index}")
        self.get_logger().info(f"Valid front beam values: {valid_values[:10]} ... (toplam {len(valid_values)})")
        self.get_logger().info(f"Calculated front average distance: {avg_distance:.2f} m")

        # MAVLink için zaman damgası (milisaniye cinsinden)
        time_boot_ms = int(time.time() * 1000) & 0xFFFFFFFF

        # DISTANCE_SENSOR mesajı gönderimi (mesafe santimetre cinsinde gönderiliyor)
        self.mav_connection.mav.distance_sensor_send(
            time_boot_ms,             # time_boot_ms
            10,                       # min_distance (cm)
            800,                      # max_distance (cm)
            int(avg_distance * 100),  # current_distance (cm)
            1,                        # type (1 = Laser)
            0,                        # id
            0,                        # orientation (0 = forward)
            0                         # covariance (0 bilinmiyor)
        )
        self.get_logger().info(f"Mesafe (ön kısım ortalama): {avg_distance:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    node = LidarMavlinkBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

