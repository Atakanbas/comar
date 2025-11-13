#!/bin/bash

# =============================================================================
# LİDAR DRIVER PARAMETRE TESTİ
# =============================================================================
# Bu script farklı LiDAR driver parametrelerini test eder
# =============================================================================

# Renk kodları
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=======================================${NC}"
echo -e "${BLUE}     LİDAR DRIVER PARAMETRE TESTİ      ${NC}"
echo -e "${BLUE}=======================================${NC}"
echo ""

# ROS2 environment'ı yükle
source /opt/ros/humble/setup.bash
source /home/enelsis-pc/enelsis_moab_ws/install/setup.bash

# Test konfigürasyonları
declare -A configs=(
    ["YDLiDAR G2"]="
ydlidar_ros2_driver_node:
  ros__parameters:
    port: /dev/ttyUSB0
    frame_id: laser_frame
    device_type: 0
    baudrate: 128000
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
    ignore_array: ''
    angle_max: 180.0
    angle_min: -180.0
    range_max: 16.0
    range_min: 0.1
    frequency: 10.0
    invalid_range_is_inf: false
    debug: false"

    ["YDLiDAR X2"]="
ydlidar_ros2_driver_node:
  ros__parameters:
    port: /dev/ttyUSB0
    frame_id: laser_frame
    device_type: 0
    baudrate: 230400
    lidar_type: 2
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
    ignore_array: ''
    angle_max: 180.0
    angle_min: -180.0
    range_max: 12.0
    range_min: 0.1
    frequency: 10.0
    invalid_range_is_inf: false
    debug: false"

    ["YDLiDAR X2L"]="
ydlidar_ros2_driver_node:
  ros__parameters:
    port: /dev/ttyUSB0
    frame_id: laser_frame
    device_type: 0
    baudrate: 128000
    lidar_type: 3
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
    ignore_array: ''
    angle_max: 180.0
    angle_min: -180.0
    range_max: 12.0
    range_min: 0.1
    frequency: 10.0
    invalid_range_is_inf: false
    debug: false"

    ["YDLiDAR X4"]="
ydlidar_ros2_driver_node:
  ros__parameters:
    port: /dev/ttyUSB0
    frame_id: laser_frame
    device_type: 0
    baudrate: 128000
    lidar_type: 4
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
    ignore_array: ''
    angle_max: 180.0
    angle_min: -180.0
    range_max: 10.0
    range_min: 0.1
    frequency: 10.0
    invalid_range_is_inf: false
    debug: false"
)

# Her konfigürasyonu test et
for config_name in "${!configs[@]}"; do
    echo -e "${YELLOW}Testing: $config_name${NC}"
    
    # Konfigürasyonu dosyaya yaz
    echo "${configs[$config_name]}" > config/lidar_test.yaml
    
    # LiDAR'ı test et
    timeout 8 ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node \
        --ros-args --params-file config/lidar_test.yaml > /tmp/lidar_test_${config_name// /_}.log 2>&1 &
    TEST_PID=$!
    
    sleep 5
    
    # Sonucu kontrol et
    if kill -0 $TEST_PID 2>/dev/null; then
        echo -e "${GREEN}✓ $config_name çalışıyor${NC}"
        
        # Topic kontrolü
        if ros2 topic list 2>/dev/null | grep -q "/scan"; then
            echo -e "${GREEN}✓ /scan topic'i oluşturuldu${NC}"
            
            # Veri kontrolü
            SCAN_COUNT=$(timeout 2 ros2 topic hz /scan 2>/dev/null | head -1 | grep -o '[0-9.]*' | head -1)
            if [ ! -z "$SCAN_COUNT" ]; then
                echo -e "${GREEN}✓ LiDAR veri hızı: ${SCAN_COUNT} Hz${NC}"
                echo -e "${GREEN}✓ BAŞARILI! $config_name konfigürasyonu çalışıyor${NC}"
                
                # Başarılı konfigürasyonu kaydet
                cp config/lidar_test.yaml config/lidar_stable.yaml
                echo -e "${GREEN}✓ Konfigürasyon kaydedildi${NC}"
                
                kill $TEST_PID 2>/dev/null
                break
            else
                echo -e "${YELLOW}⚠ Veri hızı ölçülemiyor${NC}"
            fi
        else
            echo -e "${RED}✗ /scan topic'i oluşturulamadı${NC}"
        fi
        
        kill $TEST_PID 2>/dev/null
    else
        echo -e "${RED}✗ $config_name çalışmıyor${NC}"
    fi
    
    sleep 2
    echo ""
done

echo -e "${BLUE}=======================================${NC}"
echo -e "${BLUE}     TEST TAMAMLANDI                  ${NC}"
echo -e "${BLUE}=======================================${NC}"







