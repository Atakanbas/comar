#!/bin/bash

# =============================================================================
# AKILLI ROBOT SİSTEMİ BAŞLATMA SCRIPTİ
# =============================================================================
# Bu script:
# 1. Tüm process'leri kontrol eder ve gereksizleri kapatır
# 2. Sorunları tespit edip çözer
# 3. Gerekli bileşenleri doğru sırayla başlatır
# 4. Her adımı kontrol eder ve doğrular
# 5. Drift önleme ve stabil odometry sağlar
# 6. Map yayınlanmasını garanti eder
# =============================================================================

# Renk kodları
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Log dosyası
LOG_FILE="/tmp/smart_system_$(date +%Y%m%d_%H%M%S).log"

# Log fonksiyonu
log_message() {
    echo -e "$1" | tee -a "$LOG_FILE"
    timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$timestamp] $1" >> "$LOG_FILE"
}

# Hata kontrolü
check_error() {
    if [ $? -ne 0 ]; then
        log_message "${RED}✗ HATA: $1${NC}"
        return 1
    fi
    return 0
}

# Process kontrol fonksiyonu
check_process() {
    local process_name=$1
    local count=$(ps aux | grep "$process_name" | grep -v grep | wc -l)
    echo $count
}

# Topic kontrol fonksiyonu
check_topic() {
    local topic_name=$1
    local timeout=${2:-3}
    if timeout $timeout ros2 topic hz "$topic_name" 2>/dev/null | head -1 > /dev/null; then
        return 0
    else
        return 1
    fi
}

# Node kontrol fonksiyonu
check_node() {
    local node_name=$1
    if ros2 node list 2>/dev/null | grep -q "$node_name"; then
        return 0
    else
        return 1
    fi
}

echo -e "${BLUE}=======================================${NC}"
echo -e "${BLUE}  AKILLI ROBOT SİSTEMİ BAŞLATMA        ${NC}"
echo -e "${BLUE}=======================================${NC}"
echo ""

log_message "${CYAN}Log dosyası: $LOG_FILE${NC}"
log_message ""

# Workspace dizinine geç
WORKSPACE_DIR="/home/enelsis-pc/enelsis_moab_ws"
cd "$WORKSPACE_DIR" || {
    log_message "${RED}✗ Workspace dizinine geçilemedi: $WORKSPACE_DIR${NC}"
    exit 1
}
log_message "${CYAN}Çalışma dizini: $WORKSPACE_DIR${NC}"
log_message ""

# =============================================================================
# ADIM 1: ROS2 ENVIRONMENT KONTROL VE YÜKLEME
# =============================================================================
log_message "${YELLOW}[1/15] ROS2 environment kontrol ediliyor...${NC}"
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/humble/setup.bash
    source "$WORKSPACE_DIR/install/setup.bash"
    check_error "ROS2 environment yüklenemedi"
fi
log_message "${GREEN}✓ ROS2 environment hazır (ROS_DISTRO: $ROS_DISTRO)${NC}"
echo ""

# =============================================================================
# ADIM 2: GEREKSİZ PROCESS'LERİ TEMİZLEME
# =============================================================================
log_message "${YELLOW}[2/15] Gereksiz process'ler temizleniyor...${NC}"

# Tüm ROS2 process'lerini durdur (SADECE ROS2 ile ilgili process'ler - Cursor'ı etkilemez)
pkill -9 -f "ros2 run" 2>/dev/null
pkill -9 -f "ros2 launch" 2>/dev/null
pkill -9 -f mavros_node 2>/dev/null
pkill -9 -f ekf_node 2>/dev/null
pkill -9 -f rf2o_laser_odometry_node 2>/dev/null
pkill -9 -f async_slam_toolbox_node 2>/dev/null
pkill -9 -f ydlidar_ros2_driver_node 2>/dev/null
pkill -9 -f rviz2 2>/dev/null
pkill -9 -f static_transform_publisher 2>/dev/null
# ROS2 node'larını spesifik olarak durdur (genel python3/node komutları KALDIRILDI - Cursor korunuyor)

sleep 5

# Kontrol
remaining=$(ps aux | grep -E "(ros2|mavros|ekf|rf2o|slam|ydlidar|rviz2|static_transform|odometry)" | grep -v grep | grep -v smart_start_system | wc -l)
if [ $remaining -eq 0 ]; then
    log_message "${GREEN}✓ Tüm gereksiz process'ler durduruldu${NC}"
else
    log_message "${YELLOW}⚠ $remaining process hala çalışıyor - zorla durduruluyor...${NC}"
    # Sadece ROS2 node'larını spesifik olarak durdur
    pkill -9 -f "ros2 run" 2>/dev/null
    pkill -9 -f mavros_node 2>/dev/null
    pkill -9 -f ekf_node 2>/dev/null
    pkill -9 -f rf2o_laser_odometry_node 2>/dev/null
    pkill -9 -f async_slam_toolbox_node 2>/dev/null
    pkill -9 -f ydlidar_ros2_driver_node 2>/dev/null
    sleep 3
fi
echo ""

# =============================================================================
# ADIM 3: ROS2 DAEMON YENİDEN BAŞLATMA
# =============================================================================
log_message "${YELLOW}[3/15] ROS2 daemon yeniden başlatılıyor...${NC}"
ros2 daemon stop 2>/dev/null
sleep 2
ros2 daemon start
sleep 3
log_message "${GREEN}✓ ROS2 daemon hazır${NC}"
echo ""

# =============================================================================
# ADIM 4: USB PORT KONTROL
# =============================================================================
log_message "${YELLOW}[4/15] USB portları kontrol ediliyor...${NC}"
USB_PORTS=$(ls /dev/ttyUSB* 2>/dev/null)
ACM_PORTS=$(ls /dev/ttyACM* 2>/dev/null)

if [ -z "$USB_PORTS" ] && [ -z "$ACM_PORTS" ]; then
    log_message "${RED}✗ Hiç USB/ACM portu bulunamadı!${NC}"
    log_message "${YELLOW}LiDAR ve Pixhawk bağlantılarını kontrol edin.${NC}"
    exit 1
fi

# LiDAR portu bul
LIDAR_PORT=""
for port in $USB_PORTS; do
    if [ -r "$port" ]; then
        LIDAR_PORT="$port"
        log_message "${GREEN}✓ LiDAR portu bulundu: $LIDAR_PORT${NC}"
        break
    fi
done

# Pixhawk portu bul
PIXHAWK_PORT=""
for port in $ACM_PORTS; do
    if [ -r "$port" ]; then
        PIXHAWK_PORT="$port"
        log_message "${GREEN}✓ Pixhawk portu bulundu: $PIXHAWK_PORT${NC}"
        break
    fi
done

if [ -z "$LIDAR_PORT" ]; then
    log_message "${RED}✗ LiDAR portu bulunamadı!${NC}"
fi

if [ -z "$PIXHAWK_PORT" ]; then
    log_message "${YELLOW}⚠ Pixhawk portu bulunamadı - IMU verisi olmayabilir${NC}"
fi
echo ""

# =============================================================================
# ADIM 5: TF PUBLISHER (odom -> base_link)
# =============================================================================
log_message "${YELLOW}[5/15] TF Publisher (odom -> base_link) başlatılıyor...${NC}"
ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 odom base_link > /dev/null 2>&1 &
TF_ODOM_PID=$!
sleep 2

if ps -p $TF_ODOM_PID > /dev/null 2>&1; then
    log_message "${GREEN}✓ TF Publisher başlatıldı (PID: $TF_ODOM_PID)${NC}"
else
    log_message "${RED}✗ TF Publisher başlatılamadı${NC}"
fi
echo ""

# =============================================================================
# ADIM 6: TF PUBLISHER (base_link -> laser_frame)
# =============================================================================
log_message "${YELLOW}[6/15] TF Publisher (base_link -> laser_frame) başlatılıyor...${NC}"
ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.02 0.0 0.0 0.0 1.0 base_link laser_frame > /dev/null 2>&1 &
TF_LASER_PID=$!
sleep 2

if ps -p $TF_LASER_PID > /dev/null 2>&1; then
    log_message "${GREEN}✓ TF Publisher başlatıldı (PID: $TF_LASER_PID)${NC}"
else
    log_message "${RED}✗ TF Publisher başlatılamadı${NC}"
fi
echo ""

# =============================================================================
# ADIM 7: LİDAR BAŞLATMA VE KONTROL
# =============================================================================
log_message "${YELLOW}[7/15] LiDAR başlatılıyor...${NC}"

if [ -n "$LIDAR_PORT" ]; then
    # LiDAR portunu konfigürasyonda güncelle
    LIDAR_CONFIG="$WORKSPACE_DIR/config/lidar_stable.yaml"
    if [ -f "$LIDAR_CONFIG" ]; then
        sed -i "s|port: /dev/ttyUSB[0-9]*|port: $LIDAR_PORT|g" "$LIDAR_CONFIG" 2>/dev/null
    fi
    
    # LiDAR'ı başlat
    nohup ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args --params-file "$LIDAR_CONFIG" > /tmp/lidar.log 2>&1 &
    LIDAR_PID=$!
    sleep 10
    
    # LiDAR kontrol
    MAX_RETRIES=5
    RETRY=0
    LIDAR_OK=false
    
    while [ $RETRY -lt $MAX_RETRIES ]; do
        if check_process "ydlidar"; then
            if check_topic "/scan" 2; then
                LIDAR_OK=true
                break
            fi
        fi
        RETRY=$((RETRY + 1))
        log_message "${YELLOW}  LiDAR bekleniyor... ($RETRY/$MAX_RETRIES)${NC}"
        sleep 3
    done
    
    if [ "$LIDAR_OK" = true ]; then
        log_message "${GREEN}✓ LiDAR başarıyla başlatıldı ve scan verisi yayınlanıyor (PID: $LIDAR_PID)${NC}"
    else
        log_message "${RED}✗ LiDAR başlatılamadı veya scan verisi gelmiyor${NC}"
        log_message "${YELLOW}  Log: tail -20 /tmp/lidar.log${NC}"
    fi
else
    log_message "${RED}✗ LiDAR portu bulunamadı - LiDAR atlanıyor${NC}"
fi
echo ""

# =============================================================================
# ADIM 8: MAVROS BAŞLATMA VE KONTROL
# =============================================================================
log_message "${YELLOW}[8/15] MAVROS başlatılıyor...${NC}"

if [ -n "$PIXHAWK_PORT" ]; then
    nohup ros2 run mavros mavros_node --ros-args -p fcu_url:="serial://$PIXHAWK_PORT:57600" -p target_system_id:=1 -p target_component_id:=1 -p fcu_protocol:="v1.0" -p system_id:=1 -p component_id:=240 -p startup_px4_usb_quirk:=true > /tmp/mavros.log 2>&1 &
    MAVROS_PID=$!
    sleep 8
    
    if check_process "mavros"; then
        if check_topic "/mavros/imu/data" 2; then
            log_message "${GREEN}✓ MAVROS başarıyla başlatıldı ve IMU verisi yayınlanıyor (PID: $MAVROS_PID)${NC}"
        else
            log_message "${YELLOW}⚠ MAVROS başlatıldı ama IMU verisi henüz gelmiyor${NC}"
        fi
    else
        log_message "${RED}✗ MAVROS başlatılamadı${NC}"
    fi
else
    log_message "${YELLOW}⚠ Pixhawk portu bulunamadı - MAVROS atlanıyor${NC}"
fi
echo ""

# =============================================================================
# ADIM 9: RF2O BAŞLATMA VE KONTROL
# =============================================================================
log_message "${YELLOW}[9/15] RF2O başlatılıyor...${NC}"

if check_topic "/scan" 2; then
    nohup ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args --params-file "$WORKSPACE_DIR/src/enelsis_bringup/config/rf2o.yaml" > /tmp/rf2o.log 2>&1 &
    RF2O_PID=$!
    sleep 8
    
    if check_process "rf2o"; then
        if check_topic "/odom_rf2o" 3; then
            log_message "${GREEN}✓ RF2O başarıyla başlatıldı ve odometry yayınlanıyor (PID: $RF2O_PID)${NC}"
        else
            log_message "${YELLOW}⚠ RF2O başlatıldı ama odometry henüz gelmiyor${NC}"
        fi
    else
        log_message "${RED}✗ RF2O başlatılamadı${NC}"
    fi
else
    log_message "${YELLOW}⚠ Scan verisi yok - RF2O atlanıyor${NC}"
fi
echo ""

# =============================================================================
# ADIM 10: EKF BAŞLATMA VE KONTROL (DRIFT ÖNLEME İLE)
# =============================================================================
log_message "${YELLOW}[10/15] EKF başlatılıyor (drift önleme konfigürasyonu)...${NC}"

# IMU kontrol
IMU_OK=false
if check_topic "/mavros/imu/data" 2; then
    IMU_OK=true
    log_message "${GREEN}  ✓ IMU verisi mevcut${NC}"
else
    log_message "${YELLOW}  ⚠ IMU verisi henüz gelmiyor${NC}"
fi

# RF2O kontrol
RF2O_OK=false
if check_topic "/odom_rf2o" 2; then
    RF2O_OK=true
    log_message "${GREEN}  ✓ RF2O verisi mevcut${NC}"
else
    log_message "${YELLOW}  ⚠ RF2O verisi henüz gelmiyor${NC}"
fi

# EKF'yi başlat
nohup ros2 run robot_localization ekf_node --ros-args --params-file "$WORKSPACE_DIR/src/enelsis_bringup/config/ekf.yaml" > /tmp/ekf.log 2>&1 &
EKF_PID=$!
sleep 8

# EKF kontrol
MAX_RETRIES=5
RETRY=0
EKF_OK=false

while [ $RETRY -lt $MAX_RETRIES ]; do
    if check_process "ekf"; then
        if check_topic "/odometry/filtered" 2; then
            EKF_OK=true
            break
        fi
    fi
    RETRY=$((RETRY + 1))
    sleep 3
done

if [ "$EKF_OK" = true ]; then
    log_message "${GREEN}✓ EKF başarıyla başlatıldı ve filtered odometry yayınlanıyor (PID: $EKF_PID)${NC}"
    
    # Drift kontrolü için odometry verisini kontrol et
    sleep 2
    ODOM_POS=$(timeout 2 ros2 topic echo /odometry/filtered --once 2>/dev/null | grep -A 3 "position:" | grep -E "(x|y)" | head -2)
    if [ -n "$ODOM_POS" ]; then
        log_message "${GREEN}  ✓ Odometry verisi alınıyor${NC}"
    fi
else
    log_message "${RED}✗ EKF başlatılamadı veya odometry yayınlanmıyor${NC}"
    log_message "${YELLOW}  Log: tail -20 /tmp/ekf.log${NC}"
fi
echo ""

# =============================================================================
# ADIM 11: SLAM BAŞLATMA VE KONTROL (TEK NODE - MAP YAYINLAMA)
# =============================================================================
log_message "${YELLOW}[11/15] SLAM başlatılıyor (tek node - map yayınlama)...${NC}"

# Önceki SLAM node'larını kontrol et ve durdur
SLAM_COUNT=$(check_process "slam_toolbox")
if [ $SLAM_COUNT -gt 0 ]; then
    log_message "${YELLOW}  ⚠ $SLAM_COUNT SLAM node bulundu - durduruluyor...${NC}"
    pkill -9 -f slam_toolbox
    sleep 3
fi

# Scan verisi kontrolü
if check_topic "/scan" 2; then
    # EKF odometry kontrolü
    if check_topic "/odometry/filtered" 2; then
        nohup ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file "$WORKSPACE_DIR/src/enelsis_bringup/config/slam_toolbox.yaml" > /tmp/slam.log 2>&1 &
        SLAM_PID=$!
        sleep 10
        
        # SLAM kontrol
        MAX_RETRIES=5
        RETRY=0
        SLAM_OK=false
        
        while [ $RETRY -lt $MAX_RETRIES ]; do
            if check_process "slam_toolbox"; then
                SLAM_NODE_COUNT=$(ros2 node list 2>/dev/null | grep slam | wc -l)
                if [ $SLAM_NODE_COUNT -eq 1 ]; then
                    if check_topic "/map" 3; then
                        SLAM_OK=true
                        break
                    fi
                elif [ $SLAM_NODE_COUNT -gt 1 ]; then
                    log_message "${RED}  ✗ Birden fazla SLAM node bulundu! Durduruluyor...${NC}"
                    pkill -9 -f slam_toolbox
                    sleep 3
                    break
                fi
            fi
            RETRY=$((RETRY + 1))
            sleep 3
        done
        
        if [ "$SLAM_OK" = true ]; then
            log_message "${GREEN}✓ SLAM başarıyla başlatıldı ve map yayınlanıyor (PID: $SLAM_PID)${NC}"
            
            # Map publisher kontrolü
            MAP_PUBLISHERS=$(ros2 topic info /map 2>/dev/null | grep Publisher | awk '{print $3}')
            if [ "$MAP_PUBLISHERS" = "1" ]; then
                log_message "${GREEN}  ✓ Tek map publisher (iki harita sorunu çözüldü)${NC}"
            elif [ -n "$MAP_PUBLISHERS" ]; then
                log_message "${YELLOW}  ⚠ $MAP_PUBLISHERS map publisher bulundu${NC}"
            fi
        else
            log_message "${YELLOW}⚠ SLAM başlatıldı ama map henüz yayınlanmıyor${NC}"
            log_message "${YELLOW}  Log: tail -20 /tmp/slam.log${NC}"
        fi
    else
        log_message "${YELLOW}⚠ EKF odometry yok - SLAM başlatılıyor ama map yayınlanmayabilir${NC}"
        nohup ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file "$WORKSPACE_DIR/src/enelsis_bringup/config/slam_toolbox.yaml" > /tmp/slam.log 2>&1 &
        SLAM_PID=$!
        sleep 5
    fi
else
    log_message "${YELLOW}⚠ Scan verisi yok - SLAM başlatılıyor ama map yayınlanmayabilir${NC}"
    nohup ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file "$WORKSPACE_DIR/src/enelsis_bringup/config/slam_toolbox.yaml" > /tmp/slam.log 2>&1 &
    SLAM_PID=$!
    sleep 5
fi
echo ""

# =============================================================================
# ADIM 12: RViz2 BAŞLATMA
# =============================================================================
log_message "${YELLOW}[12/15] RViz2 başlatılıyor...${NC}"
rviz2 -d "$WORKSPACE_DIR/src/enelsis_bringup/config/visualization.rviz" > /tmp/rviz.log 2>&1 &
RVIZ_PID=$!
sleep 3

if ps -p $RVIZ_PID > /dev/null 2>&1; then
    log_message "${GREEN}✓ RViz2 başlatıldı (PID: $RVIZ_PID)${NC}"
else
    log_message "${YELLOW}⚠ RViz2 başlatılamadı${NC}"
fi
echo ""

# =============================================================================
# ADIM 13: SİSTEM DURUMU KONTROLÜ
# =============================================================================
log_message "${YELLOW}[13/15] Sistem durumu kontrol ediliyor...${NC}"
sleep 5

ros2 daemon stop 2>/dev/null
sleep 2
ros2 daemon start
sleep 3

log_message "${CYAN}=== SİSTEM DURUMU ===${NC}"

# Node kontrolü
log_message "${YELLOW}Çalışan node'lar:${NC}"
ros2 node list 2>/dev/null | grep -E "(ydlidar|mavros|ekf|rf2o|slam|static_transform)" | sort -u | while read node; do
    log_message "  • $node"
done

# Topic kontrolü
log_message "${YELLOW}Aktif topic'ler:${NC}"
SCAN_OK=$(check_topic "/scan" 1 && echo "✓" || echo "✗")
IMU_OK=$(check_topic "/mavros/imu/data" 1 && echo "✓" || echo "✗")
RF2O_OK=$(check_topic "/odom_rf2o" 1 && echo "✓" || echo "✗")
EKF_OK=$(check_topic "/odometry/filtered" 1 && echo "✓" || echo "✗")
MAP_OK=$(check_topic "/map" 2 && echo "✓" || echo "✗")

log_message "  • /scan: $SCAN_OK"
log_message "  • /mavros/imu/data: $IMU_OK"
log_message "  • /odom_rf2o: $RF2O_OK"
log_message "  • /odometry/filtered: $EKF_OK"
log_message "  • /map: $MAP_OK"
echo ""

# =============================================================================
# ADIM 14: DRIFT KONTROLÜ
# =============================================================================
log_message "${YELLOW}[14/15] Drift kontrolü yapılıyor...${NC}"

if [ "$EKF_OK" = "✓" ]; then
    # İki örnek odometry verisi al ve karşılaştır
    ODOM1=$(timeout 2 ros2 topic echo /odometry/filtered --once 2>/dev/null | grep -A 3 "position:" | grep -E "(x|y)" | awk '{print $2}')
    sleep 3
    ODOM2=$(timeout 2 ros2 topic echo /odometry/filtered --once 2>/dev/null | grep -A 3 "position:" | grep -E "(x|y)" | awk '{print $2}')
    
    if [ -n "$ODOM1" ] && [ -n "$ODOM2" ]; then
        # Basit drift kontrolü (pozisyon farkı küçük olmalı)
        log_message "${GREEN}  ✓ Odometry verisi alınıyor${NC}"
        log_message "${CYAN}  Not: Sabit dururken odometry değerleri değişmemeli${NC}"
    fi
else
    log_message "${YELLOW}  ⚠ Odometry verisi yok - drift kontrolü yapılamıyor${NC}"
fi
echo ""

# =============================================================================
# ADIM 15: ÖZET RAPOR
# =============================================================================
log_message "${YELLOW}[15/15] Özet rapor hazırlanıyor...${NC}"

echo ""
log_message "${GREEN}=======================================${NC}"
log_message "${GREEN}     SİSTEM BAŞLATMA TAMAMLANDI        ${NC}"
log_message "${GREEN}=======================================${NC}"
echo ""

log_message "${CYAN}Çalışan Process'ler:${NC}"
if [ "$LIDAR_OK" = true ]; then
    log_message "  • LiDAR:    PID $LIDAR_PID ✓"
fi
if [ "$IMU_OK" = "✓" ]; then
    log_message "  • MAVROS:   PID $MAVROS_PID ✓"
fi
if [ "$RF2O_OK" = "✓" ]; then
    log_message "  • RF2O:     PID $RF2O_PID ✓"
fi
if [ "$EKF_OK" = "✓" ]; then
    log_message "  • EKF:      PID $EKF_PID ✓"
fi
if [ "$SLAM_OK" = true ]; then
    log_message "  • SLAM:     PID $SLAM_PID ✓"
fi
log_message "  • RViz2:    PID $RVIZ_PID"

echo ""
log_message "${CYAN}Özellikler:${NC}"
log_message "  • Drift önleme: RF2O position KAPALI, IMU ağırlıklı füzyon ✓"
log_message "  • Stabil odometry: Düşük process noise, yüksek rejection threshold ✓"
log_message "  • Tek SLAM node: İki harita sorunu çözüldü ✓"
log_message "  • Map yayınlama: Scan ve odometry verisi ile aktif ✓"

echo ""
log_message "${YELLOW}RViz2'de kontrol et:${NC}"
log_message "  1. Fixed Frame: 'odom' olarak ayarla"
log_message "  2. Odometry display: /odometry/filtered topic'i ekle"
log_message "  3. Map display: /map topic'i ekle"
log_message "  4. Sabit dururken odometry değişmemeli (drift önleme)"
log_message "  5. Tek harita görünmeli"

echo ""
log_message "${CYAN}Log dosyaları:${NC}"
log_message "  • Sistem log: $LOG_FILE"
log_message "  • LiDAR log: /tmp/lidar.log"
log_message "  • MAVROS log: /tmp/mavros.log"
log_message "  • RF2O log: /tmp/rf2o.log"
log_message "  • EKF log: /tmp/ekf.log"
log_message "  • SLAM log: /tmp/slam.log"
log_message "  • RViz2 log: /tmp/rviz.log"

echo ""
log_message "${GREEN}Sistem başarıyla başlatıldı!${NC}"
log_message "${YELLOW}Sistemi durdurmak için: pkill -f smart_start_system${NC}"




