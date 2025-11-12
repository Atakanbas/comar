# LiDAR Sistemi Kurulum ve Kullanım Kılavuzu

## Sistem Gereksinimleri
- Ubuntu 22.04 LTS
- ROS2 Humble
- YDLiDAR G2 cihazı
- USB bağlantısı

## Kurulum Adımları

### 1. Sistem Hazırlığı
```bash
# USB izinlerini ayarla
sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER

# Sistemi yeniden başlat veya logout/login yap
```

### 2. ROS2 Environment Kurulumu
```bash
# ROS2 Humble kurulumu (eğer yoksa)
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-rviz2
```

### 3. Workspace Build
```bash
cd /home/enelsis-pc/enelsis_moab_ws
colcon build --symlink-install
source install/setup.bash
```

## Kullanım

### Sistem Tanılama
```bash
./diagnose_lidar_system.sh
```

### LiDAR Sistemi Başlatma
```bash
./start_lidar_system.sh
```

### Manuel Başlatma
```bash
# 1. ROS2 environment yükle
source /opt/ros/humble/setup.bash
source /home/enelsis-pc/enelsis_moab_ws/install/setup.bash

# 2. LiDAR driver başlat
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args --params-file config/lidar_stable.yaml

# 3. TF publisher başlat (yeni terminal)
ros2 run tf2_ros static_transform_publisher 0 0 0.02 0 0 0 1 base_link laser_frame

# 4. RF2O başlat (yeni terminal)
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args --params-file config/rf2o_stable.yaml

# 5. SLAM başlat (yeni terminal)
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file config/slam.yaml

# 6. RViz başlat (yeni terminal)
rviz2 -d src/enelsis_bringup/config/visualization.rviz
```

## Konfigürasyon Dosyaları

### LiDAR Konfigürasyonu (`config/lidar_stable.yaml`)
- Port: `/dev/ttyUSB0` (otomatik tespit)
- Baudrate: 128000
- Frekans: 7.0 Hz
- Açı aralığı: -180° ile +180°
- Mesafe aralığı: 0.1m ile 16.0m

### RF2O Konfigürasyonu (`config/rf2o_stable.yaml`)
- Laser scan topic: `/scan`
- Odometry topic: `/odom_rf2o`
- Frekans: 15.0 Hz
- TF yayını: Aktif

### SLAM Konfigürasyonu (`config/slam.yaml`)
- Mapping modu
- Çözünürlük: 0.05m
- Maksimum lazer menzili: 16.0m
- Loop closure: Aktif

## Sorun Giderme

### LiDAR Bağlantı Sorunları
1. USB kablosunu kontrol edin
2. Cihaz izinlerini kontrol edin: `ls -la /dev/ttyUSB*`
3. Başka bir uygulama portu kullanıyor mu kontrol edin: `lsof /dev/ttyUSB0`

### ROS2 Topic Sorunları
```bash
# Topic'leri listele
ros2 topic list

# LiDAR verisini kontrol et
ros2 topic echo /scan

# Topic hızını kontrol et
ros2 topic hz /scan
```

### TF Sorunları
```bash
# TF frame'leri kontrol et
ros2 run tf2_ros tf2_echo base_link laser_frame

# TF tree'yi görüntüle
ros2 run tf2_tools view_frames
```

## Log Dosyaları
- Sistem logları: `/tmp/lidar_system_*.log`
- ROS2 logları: `~/.ros/log/`

## Performans İpuçları
- LiDAR'ı yüksek kaliteli USB kablosu ile bağlayın
- USB 3.0 portu kullanın
- Diğer USB cihazlarından uzak tutun
- Güç yönetimini devre dışı bırakın: `echo 'on' | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/power/control`

## Destek
Sorun yaşarsanız:
1. `./diagnose_lidar_system.sh` çalıştırın
2. Log dosyalarını kontrol edin
3. USB bağlantısını test edin
4. ROS2 environment'ı yeniden yükleyin











