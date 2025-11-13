# LiDAR Sistemi Kapsamlı Kurulum ve Kullanım Kılavuzu

## 📋 İçindekiler
- [Sistem Gereksinimleri](#sistem-gereksinimleri)
- [Hızlı Başlangıç](#hızlı-başlangıç)
- [Kurulum Scriptleri](#kurulum-scriptleri)
- [Kullanım](#kullanım)
- [Sorun Giderme](#sorun-giderme)
- [Konfigürasyon](#konfigürasyon)
- [Performans İpuçları](#performans-ipuçları)

## 🖥️ Sistem Gereksinimleri

### Donanım
- **İşletim Sistemi**: Ubuntu 22.04 LTS (önerilen)
- **LiDAR**: YDLiDAR G2 cihazı
- **USB**: USB 3.0 portu (önerilen)
- **RAM**: En az 4GB (8GB önerilen)
- **Disk**: En az 2GB boş alan

### Yazılım
- **ROS2**: Humble Hawksbill
- **Python**: 3.8+
- **CMake**: 3.16+
- **GCC**: 9.0+

## 🚀 Hızlı Başlangıç

### 1. İlk Kurulum
```bash
# Tam kurulum (sadece ilk kez)
./setup_lidar_system.sh
```

### 2. Sistem Başlatma
```bash
# Hızlı başlatma (kurulum sonrası)
./start_system.sh
```

### 3. Durum Kontrolü
```bash
# Sistem durumunu kontrol et
./check_lidar_system.sh
```

## 📜 Kurulum Scriptleri

### `setup_lidar_system.sh` - Tam Kurulum Scripti
Bu script ilk kurulum için kullanılır ve şunları yapar:

- ✅ USB izinlerini ayarlar
- ✅ ROS2 Humble kurulumunu kontrol eder
- ✅ Gerekli ROS2 paketlerini kurar
- ✅ Workspace'i build eder
- ✅ Konfigürasyon dosyalarını oluşturur
- ✅ USB portunu otomatik tespit eder
- ✅ LiDAR sistemini başlatır
- ✅ Sistem durumunu doğrular

**Kullanım:**
```bash
./setup_lidar_system.sh
```

### `start_system.sh` - Hızlı Başlatma Scripti
Bu script mevcut kurulumu kullanarak sistemi hızlıca başlatır:

- ✅ ROS2 environment'ı yükler
- ✅ Workspace'i kontrol eder
- ✅ Önceki süreçleri temizler
- ✅ USB portunu tespit eder
- ✅ LiDAR sistemini başlatır
- ✅ Süreçleri izler

**Kullanım:**
```bash
./start_system.sh
```

### `check_lidar_system.sh` - Durum Kontrol Scripti
Bu script sistem durumunu kontrol eder ve sorun giderme önerileri sunar:

- ✅ ROS2 environment kontrolü
- ✅ Workspace kontrolü
- ✅ USB cihazları kontrolü
- ✅ Konfigürasyon dosyaları kontrolü
- ✅ ROS2 paketleri kontrolü
- ✅ Çalışan süreçler kontrolü
- ✅ Topic'ler kontrolü
- ✅ TF frame'ler kontrolü
- ✅ USB izinleri kontrolü
- ✅ Sistem performansı kontrolü

**Kullanım:**
```bash
./check_lidar_system.sh
```

## 🎯 Kullanım

### Temel Kullanım
1. **İlk kurulum**: `./setup_lidar_system.sh`
2. **Sistem başlatma**: `./start_system.sh`
3. **Durum kontrolü**: `./check_lidar_system.sh`

### Manuel Kullanım
```bash
# 1. ROS2 environment yükle
source /opt/ros/humble/setup.bash
source /home/enelsis-pc/enelsis_moab_ws/install/setup.bash

# 2. LiDAR driver başlat
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node \
    --ros-args --params-file config/lidar_stable.yaml

# 3. TF publisher başlat (yeni terminal)
ros2 run tf2_ros static_transform_publisher \
    0 0 0.02 0 0 0 1 base_link laser_frame

# 4. RF2O başlat (yeni terminal)
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
    --ros-args --params-file config/rf2o_stable.yaml

# 5. SLAM başlat (yeni terminal)
ros2 run slam_toolbox async_slam_toolbox_node \
    --ros-args --params-file config/slam.yaml

# 6. RViz başlat (yeni terminal)
rviz2 -d src/enelsis_bringup/config/visualization.rviz
```

### Topic'leri İzleme
```bash
# Tüm topic'leri listele
ros2 topic list

# LiDAR verisini izle
ros2 topic echo /scan

# LiDAR veri hızını kontrol et
ros2 topic hz /scan

# Odometry verisini izle
ros2 topic echo /odom_rf2o

# Harita verisini izle
ros2 topic echo /map
```

### TF Frame'leri Kontrol Etme
```bash
# TF frame'leri listele
ros2 run tf2_tools view_frames

# Transform kontrol et
ros2 run tf2_ros tf2_echo base_link laser_frame

# TF tree'yi görüntüle
ros2 run tf2_tools view_frames
```

## 🔧 Sorun Giderme

### Yaygın Sorunlar

#### 1. USB Cihazı Bulunamıyor
```bash
# USB cihazlarını kontrol et
lsusb
ls -la /dev/ttyUSB*

# USB izinlerini kontrol et
groups $USER

# USB izinlerini ayarla
sudo usermod -a -G dialout,tty $USER
# Logout/login yapın
```

#### 2. ROS2 Environment Yüklenmiyor
```bash
# ROS2 environment'ı manuel yükle
source /opt/ros/humble/setup.bash
source /home/enelsis-pc/enelsis_moab_ws/install/setup.bash

# ROS2 kurulumunu kontrol et
ros2 pkg list | grep ydlidar
```

#### 3. Workspace Build Edilemiyor
```bash
# Workspace'i temizle ve yeniden build et
cd /home/enelsis-pc/enelsis_moab_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

#### 4. LiDAR Veri Gelmiyor
```bash
# USB portunu kontrol et
lsof /dev/ttyUSB0

# LiDAR konfigürasyonunu kontrol et
cat config/lidar_stable.yaml

# LiDAR driver'ı yeniden başlat
killall ydlidar_ros2_driver_node
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node \
    --ros-args --params-file config/lidar_stable.yaml
```

#### 5. TF Transform'ları Çalışmıyor
```bash
# TF publisher'ı yeniden başlat
killall static_transform_publisher
ros2 run tf2_ros static_transform_publisher \
    0 0 0.02 0 0 0 1 base_link laser_frame
```

### Log Dosyaları
- **Sistem logları**: `/tmp/lidar_system_*.log`
- **Robot logları**: `/tmp/robot_system_*.log`
- **Kontrol logları**: `/tmp/lidar_check_*.log`
- **ROS2 logları**: `~/.ros/log/`

### Detaylı Tanılama
```bash
# Sistem durumunu kontrol et
./check_lidar_system.sh

# Log dosyalarını izle
tail -f /tmp/lidar_system_*.log

# ROS2 loglarını izle
ros2 log set-severity DEBUG
```

## ⚙️ Konfigürasyon

### LiDAR Konfigürasyonu (`config/lidar_stable.yaml`)
```yaml
ydlidar_ros2_driver_node:
  ros__parameters:
    port: /dev/ttyUSB0          # USB portu
    frame_id: laser_frame       # TF frame ID
    baudrate: 230400           # Baudrate
    frequency: 10.0             # Veri frekansı (Hz)
    angle_max: 180.0            # Maksimum açı
    angle_min: -180.0           # Minimum açı
    range_max: 16.0             # Maksimum mesafe (m)
    range_min: 0.1              # Minimum mesafe (m)
    auto_reconnect: true        # Otomatik yeniden bağlanma
```

### RF2O Konfigürasyonu (`config/rf2o_stable.yaml`)
```yaml
CLaserOdometry2DNode:
  ros__parameters:
    laser_scan_topic: /scan     # LiDAR topic'i
    odom_topic: /odom_rf2o      # Odometry topic'i
    base_frame_id: base_link    # Base frame
    odom_frame_id: odom         # Odometry frame
    laser_frame_id: laser_frame # LiDAR frame
    freq: 15.0                  # Odometry frekansı
    publish_tf: true            # TF yayını
```

### SLAM Konfigürasyonu (`config/slam.yaml`)
```yaml
slam_toolbox:
  ros__parameters:
    mode: mapping               # Mapping modu
    resolution: 0.05            # Harita çözünürlüğü
    max_laser_range: 16.0       # Maksimum lazer menzili
    do_loop_closing: true       # Loop closure
    enable_interactive_mode: true # İnteraktif mod
```

## 🚀 Performans İpuçları

### Donanım Optimizasyonu
- **USB 3.0 portu kullanın**
- **Yüksek kaliteli USB kablosu kullanın**
- **LiDAR'ı diğer USB cihazlarından uzak tutun**
- **Güç yönetimini devre dışı bırakın**:
  ```bash
  echo 'on' | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/power/control
  ```

### Yazılım Optimizasyonu
- **Gereksiz süreçleri kapatın**
- **Sistem kaynaklarını izleyin**
- **Log seviyesini ayarlayın**:
  ```bash
  ros2 log set-severity WARN
  ```

### Konfigürasyon Optimizasyonu
- **LiDAR frekansını ihtiyaca göre ayarlayın**
- **SLAM çözünürlüğünü optimize edin**
- **RF2O parametrelerini fine-tune edin**

## 📞 Destek

### Sorun Bildirimi
1. `./check_lidar_system.sh` çalıştırın
2. Log dosyalarını kontrol edin
3. Sistem bilgilerini toplayın
4. Sorunu detaylı olarak açıklayın

### Yararlı Komutlar
```bash
# Sistem durumu
./check_lidar_system.sh

# Hızlı başlatma
./start_system.sh

# Tam kurulum
./setup_lidar_system.sh

# USB cihazları
lsusb

# ROS2 topic'leri
ros2 topic list

# TF frame'leri
ros2 run tf2_tools view_frames
```

---

**Not**: Bu sistem YDLiDAR G2 cihazı için optimize edilmiştir. Farklı LiDAR cihazları için konfigürasyon dosyalarını güncellemeniz gerekebilir.







