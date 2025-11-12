#!/bin/bash

# =============================================================================
# LİDAR TAMAMEN SIFIRLAMA SCRIPTİ
# =============================================================================
# Bu script LiDAR'ı tamamen sıfırlar ve yeniden başlatır
# =============================================================================

# Renk kodları
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=======================================${NC}"
echo -e "${BLUE}     LİDAR TAMAMEN SIFIRLANIYOR        ${NC}"
echo -e "${BLUE}=======================================${NC}"
echo ""

# 1. Tüm süreçleri durdur
echo -e "${YELLOW}[1/5] Tüm süreçler durduruluyor...${NC}"
killall -q ydlidar_ros2_driver_node ros2 rviz2 rf2o_laser_odometry_node async_slam_toolbox_node static_transform_publisher
sleep 3
echo -e "${GREEN}✓ Tüm süreçler durduruldu${NC}"

# 2. USB cihazını reset et
echo -e "${YELLOW}[2/5] USB cihazı reset ediliyor...${NC}"
if [ -e "/dev/ttyUSB0" ]; then
    # USB portunu unbind/bind et
    echo ttyUSB0 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/driver/unbind 2>/dev/null || true
    sleep 2
    echo ttyUSB0 | sudo tee /sys/bus/usb-serial/drivers/cp210x/bind 2>/dev/null || true
    sleep 2
    echo -e "${GREEN}✓ USB cihazı reset edildi${NC}"
else
    echo -e "${YELLOW}⚠ /dev/ttyUSB0 bulunamadı${NC}"
fi

# 3. USB izinlerini düzelt
echo -e "${YELLOW}[3/5] USB izinleri düzeltiliyor...${NC}"
sudo chmod 666 /dev/ttyUSB0 2>/dev/null || true
sudo chown root:dialout /dev/ttyUSB0 2>/dev/null || true
echo -e "${GREEN}✓ USB izinleri düzeltildi${NC}"

# 4. En basit konfigürasyonu kullan
echo -e "${YELLOW}[4/5] Konfigürasyon sıfırlanıyor...${NC}"
cat > config/lidar_stable.yaml << 'EOF'
ydlidar_ros2_driver_node:
  ros__parameters:
    port: /dev/ttyUSB0
    frame_id: laser_frame
    device_type: 0
    baudrate: 115200
    lidar_type: 1
    sample_rate: 9
    intensity_bit: 10
    abnormal_check_count: 4
    fixed_resolution: true
    reversion: true
    inverted: true
    auto_reconnect: true
    isSingleChannel: false
    intensity: false
    support_motor_dtr: false
    ignore_array: ""
    angle_max: 180.0
    angle_min: -180.0
    range_max: 16.0
    range_min: 0.1
    frequency: 10.0
    invalid_range_is_inf: false
    debug: false
EOF
echo -e "${GREEN}✓ Konfigürasyon sıfırlandı${NC}"

# 5. LiDAR'ı başlat
echo -e "${YELLOW}[5/5] LiDAR başlatılıyor...${NC}"
source /opt/ros/humble/setup.bash
source /home/enelsis-pc/enelsis_moab_ws/install/setup.bash

echo -e "${YELLOW}LiDAR test ediliyor (10 saniye)...${NC}"
timeout 10 ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node \
    --ros-args --params-file config/lidar_stable.yaml &

LIDAR_PID=$!
sleep 8

# LiDAR durumunu kontrol et
if kill -0 $LIDAR_PID 2>/dev/null; then
    echo -e "${GREEN}✓ LiDAR başlatıldı (PID: $LIDAR_PID)${NC}"
    
    # Topic kontrolü
    if ros2 topic list 2>/dev/null | grep -q "/scan"; then
        echo -e "${GREEN}✓ /scan topic'i oluşturuldu${NC}"
        
        # Veri kontrolü
        SCAN_COUNT=$(timeout 2 ros2 topic hz /scan 2>/dev/null | head -1 | grep -o '[0-9.]*' | head -1)
        if [ ! -z "$SCAN_COUNT" ]; then
            echo -e "${GREEN}✓ LiDAR veri hızı: ${SCAN_COUNT} Hz${NC}"
            echo -e "${GREEN}✓ LİDAR BAŞARIYLA ÇALIŞIYOR!${NC}"
        else
            echo -e "${YELLOW}⚠ LiDAR veri hızı ölçülemiyor${NC}"
        fi
    else
        echo -e "${RED}✗ /scan topic'i oluşturulamadı${NC}"
    fi
    
    # LiDAR'ı durdur
    kill $LIDAR_PID 2>/dev/null
    sleep 2
else
    echo -e "${RED}✗ LiDAR başlatılamadı${NC}"
fi

echo ""
echo -e "${BLUE}=======================================${NC}"
echo -e "${BLUE}     SIFIRLAMA TAMAMLANDI             ${NC}"
echo -e "${BLUE}=======================================${NC}"
echo ""
echo -e "${CYAN}Öneriler:${NC}"
echo -e "  • LiDAR'ı USB'den çıkarıp 5 saniye bekleyin"
echo -e "  • LiDAR'ı tekrar USB'ye takın"
echo -e "  • Farklı USB portu deneyin"
echo -e "  • USB kablosunu değiştirin"
echo -e "  • Sistem başlatmak için: ./start_system.sh"







