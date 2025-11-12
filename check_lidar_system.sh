#!/bin/bash

# =============================================================================
# LİDAR SİSTEMİ DURUM KONTROL VE SORUN GİDERME SCRIPTİ
# =============================================================================
# Bu script LiDAR sisteminin durumunu kontrol eder ve sorunları giderir
# =============================================================================

# Renk kodları
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Log dosyası
LOG_FILE="/tmp/lidar_check_$(date +%Y%m%d_%H%M%S).log"

# Log fonksiyonu
log_message() {
    echo -e "$1" | tee -a "$LOG_FILE"
}

# Başlık yazdırma fonksiyonu
print_header() {
    echo -e "${BLUE}=======================================${NC}"
    echo -e "${BLUE}     $1${NC}"
    echo -e "${BLUE}=======================================${NC}"
    echo ""
}

# Sistem bilgilerini yazdır
print_system_info() {
    log_message "${CYAN}Sistem Bilgileri:${NC}"
    log_message "  • İşletim Sistemi: $(lsb_release -d | cut -f2)"
    log_message "  • Kernel: $(uname -r)"
    log_message "  • Kullanıcı: $(whoami)"
    log_message "  • Workspace: /home/enelsis-pc/enelsis_moab_ws"
    log_message "  • Log Dosyası: $LOG_FILE"
    echo ""
}

# ROS2 environment kontrolü
check_ros2_environment() {
    log_message "${YELLOW}[1/10] ROS2 Environment Kontrolü${NC}"
    
    if [ -z "$ROS_DISTRO" ]; then
        log_message "${RED}✗ ROS2 environment yüklenmemiş${NC}"
        log_message "${YELLOW}Çözüm: source /opt/ros/humble/setup.bash${NC}"
        return 1
    else
        log_message "${GREEN}✓ ROS2 environment hazır (ROS_DISTRO: $ROS_DISTRO)${NC}"
        return 0
    fi
}

# Workspace kontrolü
check_workspace() {
    log_message "${YELLOW}[2/10] Workspace Kontrolü${NC}"
    
    if [ ! -d "/home/enelsis-pc/enelsis_moab_ws/install" ]; then
        log_message "${RED}✗ Workspace build edilmemiş${NC}"
        log_message "${YELLOW}Çözüm: cd /home/enelsis-pc/enelsis_moab_ws && colcon build${NC}"
        return 1
    else
        log_message "${GREEN}✓ Workspace build edilmiş${NC}"
        return 0
    fi
}

# USB cihazları kontrolü
check_usb_devices() {
    log_message "${YELLOW}[3/10] USB Cihazları Kontrolü${NC}"
    
    USB_DEVICES=$(ls /dev/ttyUSB* 2>/dev/null)
    if [ -z "$USB_DEVICES" ]; then
        log_message "${RED}✗ Hiç USB cihazı bulunamadı${NC}"
        log_message "${YELLOW}Çözüm: LiDAR'ı USB'ye bağlayın ve 'lsusb' komutu ile kontrol edin${NC}"
        lsusb | grep -i "serial\|usb\|tty" || echo "USB cihazları görünmüyor"
        return 1
    else
        log_message "${GREEN}✓ USB cihazları bulundu:${NC}"
        for device in $USB_DEVICES; do
            if lsof "$device" > /dev/null 2>&1; then
                log_message "  ${YELLOW}⚠ $device (meşgul)${NC}"
            else
                log_message "  ${GREEN}✓ $device (kullanılabilir)${NC}"
            fi
        done
        return 0
    fi
}

# Konfigürasyon dosyaları kontrolü
check_config_files() {
    log_message "${YELLOW}[4/10] Konfigürasyon Dosyaları Kontrolü${NC}"
    
    CONFIG_DIR="/home/enelsis-pc/enelsis_moab_ws/config"
    CONFIG_FILES=("lidar_stable.yaml" "rf2o_stable.yaml" "slam.yaml")
    ALL_EXIST=true
    
    for file in "${CONFIG_FILES[@]}"; do
        if [ -f "$CONFIG_DIR/$file" ]; then
            log_message "  ${GREEN}✓ $file${NC}"
        else
            log_message "  ${RED}✗ $file bulunamadı${NC}"
            ALL_EXIST=false
        fi
    done
    
    if [ "$ALL_EXIST" = true ]; then
        return 0
    else
        return 1
    fi
}

# ROS2 paketleri kontrolü
check_ros2_packages() {
    log_message "${YELLOW}[5/10] ROS2 Paketleri Kontrolü${NC}"
    
    PACKAGES=("ydlidar_ros2_driver" "rf2o_laser_odometry" "slam_toolbox")
    ALL_AVAILABLE=true
    
    for package in "${PACKAGES[@]}"; do
        if ros2 pkg list 2>/dev/null | grep -q "$package"; then
            log_message "  ${GREEN}✓ $package${NC}"
        else
            log_message "  ${RED}✗ $package bulunamadı${NC}"
            ALL_AVAILABLE=false
        fi
    done
    
    if [ "$ALL_AVAILABLE" = true ]; then
        return 0
    else
        return 1
    fi
}

# Çalışan süreçler kontrolü
check_running_processes() {
    log_message "${YELLOW}[6/10] Çalışan Süreçler Kontrolü${NC}"
    
    PROCESSES=("ydlidar_ros2_driver_node" "rf2o_laser_odometry_node" "async_slam_toolbox_node" "static_transform_publisher")
    RUNNING_COUNT=0
    
    for process in "${PROCESSES[@]}"; do
        if pgrep -f "$process" > /dev/null; then
            log_message "  ${GREEN}✓ $process çalışıyor${NC}"
            RUNNING_COUNT=$((RUNNING_COUNT + 1))
        else
            log_message "  ${YELLOW}⚠ $process çalışmıyor${NC}"
        fi
    done
    
    log_message "  ${CYAN}Çalışan süreç sayısı: $RUNNING_COUNT/${#PROCESSES[@]}${NC}"
    
    if [ $RUNNING_COUNT -eq ${#PROCESSES[@]} ]; then
        return 0
    else
        return 1
    fi
}

# Topic'ler kontrolü
check_topics() {
    log_message "${YELLOW}[7/10] ROS2 Topic'ler Kontrolü${NC}"
    
    # ROS2 environment'ı yükle
    source /opt/ros/humble/setup.bash 2>/dev/null
    source /home/enelsis-pc/enelsis_moab_ws/install/setup.bash 2>/dev/null
    
    TOPICS_OK=true
    
    if ros2 topic list 2>/dev/null | grep -q "/scan"; then
        log_message "  ${GREEN}✓ /scan topic'i aktif${NC}"
        # LiDAR veri kontrolü
        SCAN_COUNT=$(timeout 5 ros2 topic hz /scan 2>/dev/null | head -1 | grep -o '[0-9.]*' | head -1)
        if [ ! -z "$SCAN_COUNT" ]; then
            log_message "  ${GREEN}✓ LiDAR veri hızı: ${SCAN_COUNT} Hz${NC}"
        else
            log_message "  ${YELLOW}⚠ LiDAR veri hızı ölçülemiyor${NC}"
        fi
    else
        log_message "  ${RED}✗ /scan topic'i bulunamadı${NC}"
        TOPICS_OK=false
    fi

    if ros2 topic list 2>/dev/null | grep -q "/odom"; then
        log_message "  ${GREEN}✓ /odom topic'i aktif${NC}"
    else
        log_message "  ${YELLOW}⚠ /odom topic'i bulunamadı${NC}"
    fi

    if ros2 topic list 2>/dev/null | grep -q "/map"; then
        log_message "  ${GREEN}✓ /map topic'i aktif${NC}"
    else
        log_message "  ${YELLOW}⚠ /map topic'i bulunamadı${NC}"
    fi
    
    if [ "$TOPICS_OK" = true ]; then
        return 0
    else
        return 1
    fi
}

# TF frame'ler kontrolü
check_tf_frames() {
    log_message "${YELLOW}[8/10] TF Frame'ler Kontrolü${NC}"
    
    # ROS2 environment'ı yükle
    source /opt/ros/humble/setup.bash 2>/dev/null
    source /home/enelsis-pc/enelsis_moab_ws/install/setup.bash 2>/dev/null
    
    if timeout 5 ros2 run tf2_ros tf2_echo base_link laser_frame 2>/dev/null | head -1 | grep -q "Successfully"; then
        log_message "  ${GREEN}✓ base_link -> laser_frame transform mevcut${NC}"
        return 0
    else
        log_message "  ${RED}✗ base_link -> laser_frame transform bulunamadı${NC}"
        return 1
    fi
}

# USB izinleri kontrolü
check_usb_permissions() {
    log_message "${YELLOW}[9/10] USB İzinleri Kontrolü${NC}"
    
    if groups $USER | grep -q "dialout" && groups $USER | grep -q "tty"; then
        log_message "  ${GREEN}✓ USB izinleri doğru${NC}"
        return 0
    else
        log_message "  ${RED}✗ USB izinleri eksik${NC}"
        log_message "  ${YELLOW}Çözüm: sudo usermod -a -G dialout,tty \$USER${NC}"
        return 1
    fi
}

# Sistem performansı kontrolü
check_system_performance() {
    log_message "${YELLOW}[10/10] Sistem Performansı Kontrolü${NC}"
    
    # CPU kullanımı
    CPU_USAGE=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | awk -F'%' '{print $1}')
    log_message "  ${CYAN}CPU kullanımı: ${CPU_USAGE}%${NC}"
    
    # Bellek kullanımı
    MEMORY_USAGE=$(free | grep Mem | awk '{printf "%.1f", $3/$2 * 100.0}')
    log_message "  ${CYAN}Bellek kullanımı: ${MEMORY_USAGE}%${NC}"
    
    # Disk kullanımı
    DISK_USAGE=$(df -h / | awk 'NR==2{printf "%s", $5}')
    log_message "  ${CYAN}Disk kullanımı: ${DISK_USAGE}${NC}"
    
    # USB cihazı güç yönetimi
    if [ -f "/sys/bus/usb-serial/devices/ttyUSB0/power/control" ]; then
        USB_POWER=$(cat /sys/bus/usb-serial/devices/ttyUSB0/power/control 2>/dev/null)
        log_message "  ${CYAN}USB güç yönetimi: ${USB_POWER}${NC}"
    fi
    
    return 0
}

# Sorun giderme önerileri
provide_troubleshooting_tips() {
    log_message "${CYAN}=======================================${NC}"
    log_message "${CYAN}     SORUN GİDERME ÖNERİLERİ          ${NC}"
    log_message "${CYAN}=======================================${NC}"
    echo ""
    
    # ROS2 environment sorunları
    if ! check_ros2_environment > /dev/null 2>&1; then
        log_message "${YELLOW}ROS2 Environment Sorunları:${NC}"
        log_message "  • source /opt/ros/humble/setup.bash"
        log_message "  • source /home/enelsis-pc/enelsis_moab_ws/install/setup.bash"
        echo ""
    fi
    
    # Workspace sorunları
    if ! check_workspace > /dev/null 2>&1; then
        log_message "${YELLOW}Workspace Sorunları:${NC}"
        log_message "  • cd /home/enelsis-pc/enelsis_moab_ws"
        log_message "  • colcon build --symlink-install"
        echo ""
    fi
    
    # USB sorunları
    if ! check_usb_devices > /dev/null 2>&1; then
        log_message "${YELLOW}USB Sorunları:${NC}"
        log_message "  • LiDAR'ı USB'ye bağlayın"
        log_message "  • lsusb komutu ile cihazı kontrol edin"
        log_message "  • USB kablosunu değiştirin"
        echo ""
    fi
    
    # USB izin sorunları
    if ! check_usb_permissions > /dev/null 2>&1; then
        log_message "${YELLOW}USB İzin Sorunları:${NC}"
        log_message "  • sudo usermod -a -G dialout,tty \$USER"
        log_message "  • Logout/login yapın veya sistemi yeniden başlatın"
        echo ""
    fi
    
    # Konfigürasyon sorunları
    if ! check_config_files > /dev/null 2>&1; then
        log_message "${YELLOW}Konfigürasyon Sorunları:${NC}"
        log_message "  • ./setup_lidar_system.sh çalıştırın"
        echo ""
    fi
    
    # Genel öneriler
    log_message "${YELLOW}Genel Öneriler:${NC}"
    log_message "  • Sistem başlatmak için: ./start_system.sh"
    log_message "  • Tam kurulum için: ./setup_lidar_system.sh"
    log_message "  • Detaylı log için: tail -f /tmp/lidar_*.log"
    log_message "  • USB güç yönetimini devre dışı bırakın:"
    log_message "    echo 'on' | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/power/control"
    echo ""
}

# Ana fonksiyon
main() {
    print_header "LİDAR SİSTEMİ DURUM KONTROLÜ"
    
    print_system_info
    
    log_message "${CYAN}Kontrol başlatılıyor...${NC}"
    log_message ""
    
    # Kontrolleri yap
    ROS2_OK=$(check_ros2_environment && echo "true" || echo "false")
    WORKSPACE_OK=$(check_workspace && echo "true" || echo "false")
    USB_OK=$(check_usb_devices && echo "true" || echo "false")
    CONFIG_OK=$(check_config_files && echo "true" || echo "false")
    PACKAGES_OK=$(check_ros2_packages && echo "true" || echo "false")
    PROCESSES_OK=$(check_running_processes && echo "true" || echo "false")
    TOPICS_OK=$(check_topics && echo "true" || echo "false")
    TF_OK=$(check_tf_frames && echo "true" || echo "false")
    USB_PERM_OK=$(check_usb_permissions && echo "true" || echo "false")
    PERFORMANCE_OK=$(check_system_performance && echo "true" || echo "false")
    
    echo ""
    print_header "KONTROL SONUÇLARI"
    
    # Sonuçları özetle
    log_message "${CYAN}Kontrol Sonuçları:${NC}"
    log_message "  • ROS2 Environment: $([ "$ROS2_OK" = "true" ] && echo "✓" || echo "✗")"
    log_message "  • Workspace: $([ "$WORKSPACE_OK" = "true" ] && echo "✓" || echo "✗")"
    log_message "  • USB Cihazları: $([ "$USB_OK" = "true" ] && echo "✓" || echo "✗")"
    log_message "  • Konfigürasyon: $([ "$CONFIG_OK" = "true" ] && echo "✓" || echo "✗")"
    log_message "  • ROS2 Paketleri: $([ "$PACKAGES_OK" = "true" ] && echo "✓" || echo "✗")"
    log_message "  • Çalışan Süreçler: $([ "$PROCESSES_OK" = "true" ] && echo "✓" || echo "✗")"
    log_message "  • Topic'ler: $([ "$TOPICS_OK" = "true" ] && echo "✓" || echo "✗")"
    log_message "  • TF Frame'ler: $([ "$TF_OK" = "true" ] && echo "✓" || echo "✗")"
    log_message "  • USB İzinleri: $([ "$USB_PERM_OK" = "true" ] && echo "✓" || echo "✗")"
    log_message "  • Sistem Performansı: $([ "$PERFORMANCE_OK" = "true" ] && echo "✓" || echo "✗")"
    echo ""
    
    # Genel durum
    if [ "$ROS2_OK" = "true" ] && [ "$WORKSPACE_OK" = "true" ] && [ "$USB_OK" = "true" ] && [ "$CONFIG_OK" = "true" ] && [ "$PACKAGES_OK" = "true" ]; then
        if [ "$PROCESSES_OK" = "true" ] && [ "$TOPICS_OK" = "true" ] && [ "$TF_OK" = "true" ]; then
            log_message "${GREEN}=======================================${NC}"
            log_message "${GREEN}     SİSTEM TAMAMEN ÇALIŞIYOR!        ${NC}"
            log_message "${GREEN}=======================================${NC}"
        else
            log_message "${YELLOW}=======================================${NC}"
            log_message "${YELLOW}     SİSTEM HAZIR AMA ÇALIŞMIYOR      ${NC}"
            log_message "${YELLOW}=======================================${NC}"
            log_message "${YELLOW}./start_system.sh çalıştırarak başlatın${NC}"
        fi
    else
        log_message "${RED}=======================================${NC}"
        log_message "${RED}     SİSTEMDE SORUNLAR VAR!            ${NC}"
        log_message "${RED}=======================================${NC}"
        log_message "${RED}Lütfen aşağıdaki önerileri takip edin${NC}"
    fi
    
    echo ""
    provide_troubleshooting_tips
    
    log_message "${BLUE}=======================================${NC}"
    log_message "${BLUE}         KONTROL TAMAMLANDI            ${NC}"
    log_message "${BLUE}=======================================${NC}"
    log_message "${CYAN}Detaylı log: $LOG_FILE${NC}"
}

# Script'i çalıştır
main "$@"







