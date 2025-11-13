# 🚀 Sensor Fusion + High-Accuracy Mapping System

## 📋 Sistem Özeti

Bu sistem, **Pixhawk 6X IMU**, **YDLiDAR G2B**, **RF2O Laser Odometry**, **Extended Kalman Filter (EKF)** ve **SLAM Toolbox** kullanarak yüksek doğrulukta 2D haritalama yapar.

### ✅ Sistem Bileşenleri

| Bileşen | Açıklama | Port/Topic |
|---------|----------|------------|
| **Pixhawk 6X** | IMU verisi (ICM-42688-P) | `/dev/ttyACM0` (57600 baud) |
| **YDLiDAR G2B** | 360° Lazer tarama | `/dev/ttyUSB0` (115200 baud) |
| **RF2O** | Laser-based odometry | `/odom_rf2o` |
| **MAVROS** | Pixhawk iletişim köprüsü | `/mavros/imu/data` |
| **EKF** | Sensor fusion algoritması | `/odometry/filtered` |
| **SLAM Toolbox** | 2D haritalama | `/map` |

---

## 🏗️ Sistem Mimarisi

```
┌──────────────────┐
│  Pixhawk 6X      │
│  ICM-42688-P IMU │ ──► Angular Velocity (Yaw)
│                  │ ──► Linear Acceleration (X, Y)
└────────┬─────────┘
         │ /mavros/imu/data
         ▼
┌─────────────────────────────────────────┐
│   Robot Localization (EKF Node)         │
│   ┌─────────────────────────────────┐   │
│   │  State Estimation @ 50Hz        │   │
│   │  • IMU: Angular velocity        │   │◄─── /odom_rf2o
│   │  • IMU: Linear acceleration     │   │     (RF2O Odometry)
│   │  • RF2O: Position (X, Y)        │   │
│   │  • RF2O: Orientation (Yaw)      │   │
│   │  • RF2O: Velocity (X, Y)        │   │
│   └─────────────────────────────────┘   │
└────────┬────────────────────────────────┘
         │ /odometry/filtered (Fused Output)
         ▼
┌─────────────────────────────────────────┐
│        SLAM Toolbox (Async Mode)        │
│  ┌───────────────────────────────────┐  │
│  │  • Loop closure detection         │  │◄─── /scan
│  │  • Graph optimization             │  │     (LiDAR)
│  │  • High-accuracy 2D mapping       │  │
│  └───────────────────────────────────┘  │
└────────┬────────────────────────────────┘
         │ /map (Occupancy Grid)
         ▼
     [Yüksek Doğrulukta Harita]
```

---

## 🔧 Kurulum ve Gereksinimler

### Donanım

- ✅ **Pixhawk 6X Holybro** (USB bağlantısı)
- ✅ **YDLiDAR G2B** (USB/Serial bağlantısı)
- ✅ **Ubuntu 22.04 LTS** (Jammy)

### Yazılım (Zaten Kurulu!)

```bash
# ROS2 Humble
sudo apt install ros-humble-desktop

# MAVROS
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# Robot Localization
sudo apt install ros-humble-robot-localization

# SLAM Toolbox
sudo apt install ros-humble-slam-toolbox

# TF2 Tools
sudo apt install ros-humble-tf2-tools ros-humble-tf2-ros

# RViz2
sudo apt install ros-humble-rviz2
```

### Workspace Build

```bash
cd /home/enelsis-pc/enelsis_moab_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Hızlı Başlangıç

### 1. Sistemi Başlatma

```bash
cd /home/enelsis-pc/enelsis_moab_ws
./start_fusion_system.sh
```

**Bu script otomatik olarak şunları başlatır:**
- YDLiDAR sürücüsü
- RF2O Laser Odometry
- MAVROS (Pixhawk IMU)
- Robot Localization (EKF)
- SLAM Toolbox
- Gerekli TF yayınları

### 2. Sistem Kontrolü

```bash
./check_fusion_system.sh
```

**Kontrol edilen öğeler:**
- ✅ Donanım bağlantıları
- ✅ ROS2 node'ları
- ✅ Topic'ler ve frekansları
- ✅ TF frame'leri
- ✅ Diagnostics
- ✅ CPU/Memory kullanımı

### 3. Haritayı Görüntüleme

```bash
rviz2
```

**RViz2 Konfigürasyonu:**
1. Fixed Frame: `map`
2. Add → Map → Topic: `/map`
3. Add → LaserScan → Topic: `/scan`
4. Add → Odometry → Topic: `/odometry/filtered`
5. Add → TF → Tüm frame'leri göster

---

## 📊 Önemli Topic'ler

| Topic | Mesaj Tipi | Frekans | Açıklama |
|-------|-----------|---------|----------|
| `/scan` | `sensor_msgs/LaserScan` | ~10 Hz | LiDAR tarama verisi |
| `/odom_rf2o` | `nav_msgs/Odometry` | ~20 Hz | RF2O odometry |
| `/mavros/imu/data` | `sensor_msgs/Imu` | ~50 Hz | Pixhawk IMU verisi |
| `/odometry/filtered` | `nav_msgs/Odometry` | 50 Hz | EKF füzyon çıktısı |
| `/map` | `nav_msgs/OccupancyGrid` | ~1 Hz | SLAM haritası |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | Sistem diagnostics |

### Topic İnceleme Komutları

```bash
# Topic listesini göster
ros2 topic list

# Topic bilgisini göster
ros2 topic info /odometry/filtered

# Topic verisini göster
ros2 topic echo /odometry/filtered

# Topic frekansını ölç
ros2 topic hz /mavros/imu/data

# Topic bant genişliğini ölç
ros2 topic bw /scan
```

---

## ⚙️ Konfigürasyon Dosyaları

### 1. EKF Konfigürasyonu (`ekf.yaml`)

**Lokasyon:** `/home/enelsis-pc/enelsis_moab_ws/src/enelsis_bringup/config/ekf.yaml`

**Önemli parametreler:**
```yaml
frequency: 50.0  # IMU için yüksek frekans
two_d_mode: true  # 2D yer robotu

# IMU konfigürasyonu
imu0: /mavros/imu/data
imu0_config: [false, false, false,  # position - kullanma
              false, false, true,   # orientation - sadece yaw
              false, false, false,  # velocity - kullanma  
              false, false, true,   # angular velocity - sadece yaw
              true, true, false]    # acceleration - x,y kullan

# RF2O konfigürasyonu
odom0: /odom_rf2o
odom0_config: [true, true, false,   # position - x,y kullan
               false, false, true,  # orientation - sadece yaw
               true, true, false,   # velocity - x,y kullan
               false, false, true,  # angular velocity - yaw
               false, false, false] # acceleration - kullanma
```

### 2. MAVROS Konfigürasyonu (`mavros.yaml`)

**Lokasyon:** `/home/enelsis-pc/enelsis_moab_ws/src/enelsis_bringup/config/mavros.yaml`

**Önemli parametreler:**
```yaml
fcu_url: "serial:///dev/ttyACM0:57600"  # Pixhawk USB bağlantısı
target_system_id: 1
target_component_id: 1
fcu_protocol: "v1.0"  # MAVLink v1.0

# IMU Plugin
imu:
  frame_id: "base_link"
  orientation_stdev: 0.01
  angular_velocity_stdev: 0.01
  linear_acceleration_stdev: 0.01
```

### 3. RF2O Konfigürasyonu (`rf2o.yaml`)

**Lokasyon:** `/home/enelsis-pc/enelsis_moab_ws/src/enelsis_bringup/config/rf2o.yaml`

**Önemli parametreler:**
```yaml
laser_scan_topic: /scan
odom_topic: /odom_rf2o
base_frame_id: base_link
odom_frame_id: odom
publish_tf: true
freq: 20.0  # 20 Hz odometry
```

### 4. SLAM Toolbox Konfigürasyonu (`slam_toolbox.yaml`)

**Lokasyon:** `/home/enelsis-pc/enelsis_moab_ws/src/enelsis_bringup/config/slam_toolbox.yaml`

**Önemli parametreler:**
```yaml
mode: mapping  # async mapping mode
scan_topic: /scan
odom_topic: /odometry/filtered  # EKF çıktısı kullanılıyor
map_frame: map
base_frame: base_link
odom_frame: odom

# Loop closure
do_loop_closing: true
loop_search_maximum_distance: 3.0
```

---

## 🔍 Troubleshooting

### Pixhawk Bağlantı Sorunu

```bash
# Pixhawk port kontrolü
ls -l /dev/ttyACM0

# USB bağlantısını göster
lsusb | grep Auterion

# MAVROS loglarını kontrol et
ros2 run mavros mavros_node --ros-args --log-level debug
```

**Çözüm:**
```bash
# Serial port izinleri
sudo chmod 666 /dev/ttyACM0

# Dialout grubuna ekleme
sudo usermod -a -G dialout $USER
# Çıkış yapıp tekrar giriş yapın
```

### LiDAR Bağlantı Sorunu

```bash
# LiDAR port kontrolü
ls -l /dev/ttyUSB0

# LiDAR loglarını kontrol et
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args --log-level debug
```

**Çözüm:**
```bash
# USB device reset
sudo usbreset /dev/ttyUSB0

# Port izinleri
sudo chmod 666 /dev/ttyUSB0
```

### EKF Füzyon Sorunu

```bash
# EKF diagnostics kontrol
ros2 topic echo /diagnostics | grep ekf

# IMU verisi kontrol
ros2 topic hz /mavros/imu/data

# RF2O verisi kontrol
ros2 topic hz /odom_rf2o

# EKF çıktısı kontrol
ros2 topic hz /odometry/filtered
```

**Olası sorunlar:**
1. IMU verisi gelmiyor → MAVROS bağlantısını kontrol et
2. RF2O verisi gelmiyor → LiDAR `/scan` topic'ini kontrol et
3. EKF çıktısı yok → Her iki input'u da kontrol et

### TF Frame Sorunu

```bash
# TF tree görüntüle
ros2 run tf2_tools view_frames

# Belirli bir transform'u kontrol et
ros2 run tf2_ros tf2_echo map base_link

# TF frame listesi
ros2 run tf2_ros tf2_monitor
```

**Beklenen frame hierarchy:**
```
map → odom → base_link → laser_frame
               └─→ base_footprint
```

---

## 📈 Performans İpuçları

### 1. IMU Kalite Kontrolü

```bash
# IMU mesaj frekansını ölç
ros2 topic hz /mavros/imu/data
# Beklenen: ~50-100 Hz

# IMU verisini göster
ros2 topic echo /mavros/imu/data --once
# angular_velocity ve linear_acceleration kontrolü
```

### 2. RF2O Odometry Kalitesi

```bash
# RF2O frekansını ölç
ros2 topic hz /odom_rf2o
# Beklenen: ~20 Hz

# LiDAR kalitesini kontrol et
ros2 topic hz /scan
# Beklenen: ~10 Hz, 530 nokta/scan
```

### 3. EKF Füzyon Kalitesi

```bash
# EKF frekansını ölç
ros2 topic hz /odometry/filtered
# Beklenen: 50 Hz

# EKF diagnostics
ros2 topic echo /diagnostics | grep ekf
# level: OK olmalı
```

### 4. SLAM Harita Kalitesi

```bash
# Map topic kontrolü
ros2 topic info /map

# Map güncellemelerini izle
ros2 topic echo /map --once | grep -A 5 "info"

# Loop closure sayısı
ros2 topic echo /slam_toolbox/feedback
```

---

## 🎯 Kullanım Senaryoları

### Senaryo 1: Basit Haritalama

```bash
# Sistemi başlat
./start_fusion_system.sh

# RViz'de haritayı görüntüle
rviz2 &

# Robotu yavaşça hareket ettir
# SLAM otomatik olarak harita oluşturacak

# Haritayı kaydet
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

### Senaryo 2: Yüksek Doğrulukta Haritalama

```bash
# Sistemi başlat
./start_fusion_system.sh

# EKF füzyon kalitesini kontrol et
ros2 topic echo /diagnostics | grep ekf

# Loop closure aktif olduğundan emin ol
ros2 param get /slam_toolbox do_loop_closing

# Aynı alanları tekrar ziyaret ederek loop closure tetikle
# Bu haritanın global tutarlılığını artırır

# Haritayı kaydet
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/enelsis-pc/high_accuracy_map'}}"
```

### Senaryo 3: Gerçek Zamanlı Lokalizasyon

```bash
# Sistemi localization mode'da başlat
ros2 launch enelsis_bringup fusion_mapping.launch.py

# Daha önce kaydettiğiniz haritayı yükleyin
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/enelsis-pc/my_map.yaml

# SLAM Toolbox'u localization mode'a geçir
ros2 param set /slam_toolbox mode localization

# Robot artık kaydedilmiş haritada lokalize olacak
```

---

## 📚 Ek Kaynaklar

### ROS2 Komutları

```bash
# Node listesi
ros2 node list

# Node bilgisi
ros2 node info /ekf_filter_node

# Parameter listesi
ros2 param list /ekf_filter_node

# Parameter değeri
ros2 param get /ekf_filter_node frequency

# Parameter değiştir
ros2 param set /ekf_filter_node frequency 60.0

# Service listesi
ros2 service list

# Service çağır
ros2 service call /reset_ekf std_srvs/srv/Empty
```

### Loglar

```bash
# ROS2 log seviyesi
ros2 run <package> <node> --ros-args --log-level debug

# System logs
journalctl -u ros2-daemon
```

### Bag Kaydetme (Veri Toplama)

```bash
# Tüm topic'leri kaydet
ros2 bag record -a

# Belirli topic'leri kaydet
ros2 bag record /scan /odom_rf2o /mavros/imu/data /odometry/filtered /map

# Bag oynat
ros2 bag play my_bag_file
```

---

## 🤝 Destek ve Katkı

### Sorun Bildirme

Sistem ile ilgili sorunlar için:
1. `./check_fusion_system.sh` çalıştırın
2. Logları kontrol edin
3. İlgili topic'leri inceleyin

### Performans İyileştirme

1. **IMU frekansını artırma:**
   - `ekf.yaml` → `frequency: 100.0`
   
2. **RF2O hassasiyetini artırma:**
   - `rf2o.yaml` → `freq: 30.0`
   
3. **SLAM loop closure hassasiyeti:**
   - `slam_toolbox.yaml` → `minimum_travel_distance: 0.2`

---

## 📝 Sürüm Geçmişi

- **v1.0** (2025-10-21): İlk sürüm
  - Pixhawk 6X IMU entegrasyonu
  - RF2O + LiDAR odometry
  - EKF sensor fusion
  - SLAM Toolbox entegrasyonu

---

## 📄 Lisans

Bu proje [GPLv3](LICENSE) altında lisanslanmıştır.

---

## 🎓 Referanslar

- [MAVROS Documentation](http://wiki.ros.org/mavros)
- [Robot Localization Package](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [RF2O Laser Odometry](https://github.com/MAPIRlab/rf2o_laser_odometry)
- [YDLiDAR ROS2 Driver](https://github.com/YDLIDAR/ydlidar_ros2_driver)

---

**🚀 İyi Haritalamalar!**









