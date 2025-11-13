#!/bin/bash
# RF2O + IMU + EKF + SLAM Pipeline
# YDLIDAR G2 → /scan
# Pixhawk 6X IMU → /mavros/imu/data
# RF2O → /rf2o/odom
# EKF → /odometry/filtered → /odom → /base_link
# SLAM Toolbox → /map

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

WORKSPACE_DIR="/home/enelsis-pc/enelsis_moab_ws"
CONFIG_DIR="$WORKSPACE_DIR/config"
declare -a PIDS=()

log_message() {
    echo -e "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

cleanup_and_exit() {
    log_message "${YELLOW}Sistem kapatılıyor...${NC}"
    
    # Önce tüm PIDS'leri durdur
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -15 "$pid" 2>/dev/null
            sleep 0.5
            if kill -0 "$pid" 2>/dev/null; then
                kill -9 "$pid" 2>/dev/null
            fi
        fi
    done
    
    # Tüm ROS2 node'larını temizle
    log_message "${YELLOW}  → ROS2 node'ları durduruluyor...${NC}"
    killall -9 ydlidar_ros2_driver_node 2>/dev/null
    killall -9 rf2o_laser_odometry_node 2>/dev/null
    killall -9 mavros_node 2>/dev/null
    killall -9 mavros_router 2>/dev/null
    killall -9 ekf_node 2>/dev/null
    killall -9 async_slam_toolbox_node 2>/dev/null
    killall -9 static_transform_publisher 2>/dev/null
    killall -9 rviz2 2>/dev/null
    
    sleep 1
    log_message "${GREEN}✓ Sistem temizlendi${NC}"
    exit 0
}

trap cleanup_and_exit SIGINT SIGTERM

clear
log_message "${CYAN}╔════════════════════════════════════════════════════╗${NC}"
log_message "${CYAN}║  RF2O + IMU + EKF + SLAM PIPELINE                 ║${NC}"
log_message "${CYAN}║  YDLIDAR G2 → RF2O → EKF → SLAM Toolbox           ║${NC}"
log_message "${CYAN}╚════════════════════════════════════════════════════╝${NC}"
echo ""

source /opt/ros/humble/setup.bash
cd "$WORKSPACE_DIR"
source install/setup.bash

# 1. YDLIDAR G2
log_message "${BLUE}[1/7] YDLIDAR G2 başlatılıyor...${NC}"
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args --params-file "$CONFIG_DIR/lidar_stable.yaml" &
PIDS+=($!)
sleep 5

# 2. TF: base_link → laser_frame
log_message "${BLUE}[2/7] TF (base_link → laser_frame) başlatılıyor...${NC}"
ros2 run tf2_ros static_transform_publisher 0 0 0.15 0 0 0 base_link laser_frame &
PIDS+=($!)
sleep 1

# 3. TF: base_link → imu_link
log_message "${BLUE}[3/7] TF (base_link → imu_link) başlatılıyor...${NC}"
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link imu_link &
PIDS+=($!)
sleep 1

# 4. MAVROS - Pixhawk 6X IMU
log_message "${BLUE}[4/7] MAVROS başlatılıyor (Pixhawk 6X IMU)...${NC}"
ros2 run mavros mavros_node --ros-args \
    -p fcu_url:="serial:///dev/ttyACM0:921600" \
    -p plugin_allowlist:="['sys_*', 'imu']" \
    -p imu.frame_id:="imu_link" \
    -p imu.linear_acceleration_stdev:=0.0003 \
    -p imu.angular_velocity_stdev:=0.0003490659 \
    -p imu.orientation_stdev:=0.0087 \
    -p imu.magnetic_stdev:=0.0 &
PIDS+=($!)
sleep 10

# 5. RF2O Laser Odometry
log_message "${BLUE}[5/7] RF2O Laser Odometry başlatılıyor...${NC}"
sleep 2
if timeout 2 ros2 topic echo /scan --once > /dev/null 2>&1; then
    log_message "${GREEN}  ✓ /scan topic'i aktif${NC}"
    ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args \
        --params-file "$WORKSPACE_DIR/src/enelsis_bringup/config/rf2o_odom.yaml" &
    PIDS+=($!)
    sleep 5
    
    # RF2O kontrolü
    if timeout 2 ros2 topic echo /rf2o/odom --once > /dev/null 2>&1; then
        log_message "${GREEN}  ✓ RF2O odometry yayınlanıyor (/rf2o/odom)${NC}"
    else
        log_message "${YELLOW}  ⚠ RF2O odometry henüz yayınlanmıyor${NC}"
    fi
else
    log_message "${RED}  ✗ /scan topic'i yok! LiDAR'ı kontrol edin.${NC}"
fi

# 6. EKF Filter (RF2O + IMU fusion)
log_message "${BLUE}[6/7] EKF Filter başlatılıyor (RF2O + IMU fusion)...${NC}"
sleep 3
if timeout 2 ros2 topic echo /mavros/imu/data --once > /dev/null 2>&1; then
    log_message "${GREEN}  ✓ IMU verisi mevcut${NC}"
    ros2 run robot_localization ekf_node --ros-args \
        --params-file "$WORKSPACE_DIR/src/enelsis_bringup/config/ekf_rf2o_imu.yaml" &
    PIDS+=($!)
    sleep 5
    
    # EKF kontrolü
    if timeout 2 ros2 topic echo /odometry/filtered --once > /dev/null 2>&1; then
        log_message "${GREEN}  ✓ EKF odometry yayınlanıyor (/odometry/filtered)${NC}"
    else
        log_message "${YELLOW}  ⚠ EKF odometry henüz yayınlanmıyor${NC}"
    fi
else
    log_message "${YELLOW}  ⚠ IMU verisi henüz gelmiyor, EKF başlatılıyor...${NC}"
    ros2 run robot_localization ekf_node --ros-args \
        --params-file "$WORKSPACE_DIR/src/enelsis_bringup/config/ekf_rf2o_imu.yaml" &
    PIDS+=($!)
    sleep 5
fi

# 7. SLAM Toolbox
log_message "${BLUE}[7/7] SLAM Toolbox başlatılıyor...${NC}"
sleep 3
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
    --params-file "$WORKSPACE_DIR/src/enelsis_bringup/config/slam_toolbox.yaml" \
    -r scan:=/scan \
    -r odom:=/odometry/filtered &
PIDS+=($!)
sleep 5

log_message "${GREEN}✓ SİSTEM BAŞLATILDI!${NC}"
echo ""
log_message "${CYAN}Topic'ler:${NC}"
log_message "  • /scan → YDLIDAR G2"
log_message "  • /mavros/imu/data → Pixhawk 6X IMU"
log_message "  • /rf2o/odom → RF2O Laser Odometry"
log_message "  • /odometry/filtered → EKF Fusion (RF2O + IMU)"
log_message "  • /map → SLAM Toolbox Map"
echo ""
log_message "${CYAN}TF Zinciri:${NC}"
log_message "  map → odom → base_link → laser_frame"
log_message "  map → odom → base_link → imu_link"
echo ""
log_message "${YELLOW}Sistem çalışıyor... (Ctrl+C ile durdurun)${NC}"

# Bekleme döngüsü
while true; do
    sleep 1
done



