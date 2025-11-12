# RViz2 Sensor Fusion Görselleştirme Rehberi

## 🎯 Genel Bakış

Bu RViz2 konfigürasyonu, Pixhawk 6X IMU + YDLiDAR + RF2O + EKF sensor fusion sisteminin tüm verilerini görselleştirmek için tasarlanmıştır.

## 🚀 Kullanım

### 1. Sistemi Başlat
```bash
# Terminal 1: Sensor fusion sistemini başlat
./start_fusion_system.sh

# Terminal 2: RViz2'yi başlat
./start_rviz.sh
```

### 2. Manuel Başlatma
```bash
# RViz2'yi manuel olarak başlat
rviz2 -d config/fusion_mapping.rviz
```

## 📊 Görüntülenen Veriler

### 🔴 **LaserScan** (`/scan`)
- **Renk**: Beyaz/Kırmızı noktalar
- **Açıklama**: YDLiDAR'dan gelen lazer tarama verileri
- **Görsel**: Gerçek zamanlı nokta bulutu
- **Kullanım**: Çevre haritalama ve engel tespiti

### 🗺️ **Map** (`/map`)
- **Renk**: Gri tonları
- **Açıklama**: SLAM Toolbox tarafından oluşturulan harita
- **Görsel**: Grid tabanlı harita
- **Kullanım**: Navigasyon ve konum belirleme

### 🟡 **IMU** (`/mavros/imu/data`)
- **Renk**: Sarı oklar
- **Açıklama**: Pixhawk 6X IMU orientasyon verileri
- **Görsel**: 3D koordinat sistemi (X, Y, Z eksenleri)
- **Kullanım**: Robot orientasyonunu takip etme

### 🔴 **Odometry** (`/odometry/filtered`)
- **Renk**: Kırmızı oklar
- **Açıklama**: EKF sensor fusion çıktısı
- **Görsel**: Robot pozisyonu ve yönü
- **Kullanım**: Füzyon edilmiş pozisyon takibi

### 🟢 **Path** (`/odometry/filtered`)
- **Renk**: Yeşil çizgi
- **Açıklama**: Robot'un takip ettiği yol
- **Görsel**: Sürekli çizgi
- **Kullanım**: Hareket geçmişi

### 🔵 **TF Frames**
- **Renk**: Çok renkli koordinat sistemleri
- **Açıklama**: Transform frame'leri
- **Görsel**: 3D eksenler (X=Kırmızı, Y=Yeşil, Z=Mavi)
- **Frames**: `map`, `odom`, `base_link`, `laser_frame`

### 🤖 **Robot Model**
- **Renk**: Varsayılan robot modeli
- **Açıklama**: 3D robot modeli
- **Görsel**: URDF tabanlı model
- **Kullanım**: Robot görünümü

## 🎮 Kontroller

### Mouse Kontrolleri
- **Sol Tık + Sürükle**: Kamera döndürme
- **Orta Tık + Sürükle**: Kamera kaydırma
- **Sağ Tık + Sürükle**: Zoom in/out
- **Scroll**: Zoom

### Klavye Kısayolları
- **R**: Kamera sıfırlama
- **F**: Seçili objeye odaklanma
- **G**: Grid göster/gizle

## 🔧 Konfigürasyon Ayarları

### Display Ayarları
- **Fixed Frame**: `map` (ana koordinat sistemi)
- **Frame Rate**: 30 FPS
- **Background**: Koyu gri

### LiDAR Ayarları
- **Size**: 3 pixels
- **Color Transformer**: Intensity
- **Style**: Flat Squares

### IMU Ayarları
- **Axes Length**: 1.0
- **Color**: Sarı
- **History Length**: 1

### Odometry Ayarları
- **History Length**: 100
- **Shape**: Arrow
- **Color**: Kırmızı

## 🐛 Sorun Giderme

### Topic Bulunamıyor
```bash
# Topic'leri kontrol et
ros2 topic list

# Belirli topic'i kontrol et
ros2 topic echo /scan --once
```

### TF Frame Eksik
```bash
# TF tree'yi kontrol et
ros2 run tf2_tools view_frames

# TF frame'leri listele
ros2 run tf2_ros tf2_echo map base_link
```

### RViz2 Açılmıyor
```bash
# ROS2 environment'ı kontrol et
echo $ROS_DISTRO

# Konfigürasyon dosyasını kontrol et
ls -la config/fusion_mapping.rviz
```

## 📈 Performans İpuçları

1. **Yüksek FPS için**: Gereksiz display'leri kapat
2. **Bellek tasarrufu için**: History length'i azalt
3. **Görsel kalite için**: Point size'ı artır
4. **Debug için**: TF frame'leri göster

## 🎯 Test Senaryoları

### 1. IMU Testi
- Pixhawk'ı farklı açılarda hareket ettir
- Sarı IMU oklarının değiştiğini gözlemle

### 2. LiDAR Testi
- Robot'u hareket ettir
- Kırmızı noktaların değiştiğini gözlemle

### 3. Haritalama Testi
- Robot'u yavaşça hareket ettir
- Gri haritanın oluştuğunu gözlemle

### 4. Sensor Fusion Testi
- Hem IMU hem LiDAR'ı hareket ettir
- Kırmızı odometry oklarının değiştiğini gözlemle

## 📝 Notlar

- **Fixed Frame**: `map` olarak ayarlanmıştır
- **Sync Mode**: LaserScan ile senkronize
- **Update Rate**: 30 Hz
- **Memory Usage**: ~100MB (tipik kullanım)

## 🔗 İlgili Dosyalar

- `config/fusion_mapping.rviz` - RViz2 konfigürasyonu
- `start_rviz.sh` - RViz2 başlatma scripti
- `start_fusion_system.sh` - Ana sistem başlatma scripti








