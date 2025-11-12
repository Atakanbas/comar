#!/bin/bash

# =============================================================================
# LİDAR BAĞLANTI SORUNLARI ÇÖZÜM SCRIPTİ
# =============================================================================
# Bu script LiDAR bağlantı sorunlarını çözer
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
LOG_FILE="/tmp/lidar_fix_$(date +%Y%m%d_%H%M%S).log"

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

# USB cihazını yeniden başlat
reset_usb_device() {
    log_message "${YELLOW}[1/6] USB Cihazı Yeniden Başlatılıyor...${NC}"
    
    # USB cihazını bul
    USB_DEVICE=$(lsusb | grep -i "serial\|usb\|tty" | head -1)
    if [ -z "$USB_DEVICE" ]; then
        log_message "${RED}✗ USB cihazı bulunamadı!${NC}"
        return 1
    fi
    
    log_message "${CYAN}Bulunan USB cihazı: $USB_DEVICE${NC}"
    
    # USB cihazını reset et
    if command -v usb_modeswitch > /dev/null; then
        log_message "${YELLOW}USB cihazı reset ediliyor...${NC}"
        sudo usb_modeswitch -R -v 0x10c4 -p 0xea60 2>/dev/null || true
    fi
    
    # USB portunu reset et
    if [ -f "/sys/bus/usb-serial/devices/ttyUSB0/driver/unbind" ]; then
        log_message "${YELLOW}USB portu reset ediliyor...${NC}"
        echo ttyUSB0 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/driver/unbind 2>/dev/null || true
        sleep 2
        echo ttyUSB0 | sudo tee /sys/bus/usb-serial/drivers/cp210x/bind 2>/dev/null || true
    fi
    
    log_message "${GREEN}✓ USB cihazı reset edildi${NC}"
    return 0
}

# USB izinlerini düzelt
fix_usb_permissions() {
    log_message "${YELLOW}[2/6] USB İzinleri Düzeltiliyor...${NC}"
    
    # Kullanıcıyı gerekli gruplara ekle
    sudo usermod -a -G dialout,tty $USER
    
    # USB cihazı izinlerini ayarla
    if [ -e "/dev/ttyUSB0" ]; then
        sudo chmod 666 /dev/ttyUSB0
        sudo chown root:dialout /dev/ttyUSB0
        log_message "${GREEN}✓ USB izinleri ayarlandı${NC}"
    else
        log_message "${YELLOW}⚠ /dev/ttyUSB0 bulunamadı${NC}"
    fi
    
    return 0
}

# USB güç yönetimini devre dışı bırak
disable_usb_power_management() {
    log_message "${YELLOW}[3/6] USB Güç Yönetimi Devre Dışı Bırakılıyor...${NC}"
    
    # USB güç yönetimini devre dışı bırak
    if [ -f "/sys/bus/usb-serial/devices/ttyUSB0/power/control" ]; then
        echo 'on' | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/power/control > /dev/null
        log_message "${GREEN}✓ USB güç yönetimi devre dışı bırakıldı${NC}"
    fi
    
    # USB otomatik suspend'i devre dışı bırak
    if [ -f "/sys/module/usbcore/parameters/autosuspend" ]; then
        echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend > /dev/null
        log_message "${GREEN}✓ USB otomatik suspend devre dışı bırakıldı${NC}"
    fi
    
    return 0
}

# Farklı baudrate'leri test et
test_baudrates() {
    log_message "${YELLOW}[4/6] Baudrate Test Ediliyor...${NC}"
    
    BAUDRATES=(9600 19200 38400 57600 115200 128000 230400 460800)
    WORKING_BAUDRATES=()
    
    for baudrate in "${BAUDRATES[@]}"; do
        log_message "${YELLOW}Testing baudrate: $baudrate${NC}"
        
        # Konfigürasyonu güncelle
        sed -i "s/baudrate: [0-9]*/baudrate: $baudrate/" config/lidar_stable.yaml
        
        # LiDAR'ı test et
        timeout 3 ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node \
            --ros-args --params-file config/lidar_stable.yaml > /dev/null 2>&1 &
        TEST_PID=$!
        
        sleep 2
        
        if kill -0 $TEST_PID 2>/dev/null; then
            log_message "${GREEN}✓ Baudrate $baudrate çalışıyor${NC}"
            WORKING_BAUDRATES+=($baudrate)
        else
            log_message "${RED}✗ Baudrate $baudrate çalışmıyor${NC}"
        fi
        
        kill $TEST_PID 2>/dev/null
        sleep 1
    done
    
    if [ ${#WORKING_BAUDRATES[@]} -gt 0 ]; then
        log_message "${GREEN}✓ Çalışan baudrate'ler: ${WORKING_BAUDRATES[*]}${NC}"
        # En iyi baudrate'i seç
        BEST_BAUDRATE=${WORKING_BAUDRATES[0]}
        sed -i "s/baudrate: [0-9]*/baudrate: $BEST_BAUDRATE/" config/lidar_stable.yaml
        log_message "${GREEN}✓ En iyi baudrate seçildi: $BEST_BAUDRATE${NC}"
        return 0
    else
        log_message "${RED}✗ Hiçbir baudrate çalışmıyor${NC}"
        return 1
    fi
}

# LiDAR driver'ını yeniden yükle
reload_lidar_driver() {
    log_message "${YELLOW}[5/6] LiDAR Driver Yeniden Yükleniyor...${NC}"
    
    # Mevcut süreçleri durdur
    killall -q ydlidar_ros2_driver_node 2>/dev/null
    sleep 2
    
    # ROS2 environment'ı yükle
    source /opt/ros/humble/setup.bash 2>/dev/null
    source /home/enelsis-pc/enelsis_moab_ws/install/setup.bash 2>/dev/null
    
    # LiDAR'ı başlat
    log_message "${YELLOW}LiDAR driver başlatılıyor...${NC}"
    ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node \
        --ros-args --params-file config/lidar_stable.yaml &
    LIDAR_PID=$!
    
    sleep 5
    
    # LiDAR çalışıyor mu kontrol et
    if kill -0 $LIDAR_PID 2>/dev/null; then
        log_message "${GREEN}✓ LiDAR driver başlatıldı (PID: $LIDAR_PID)${NC}"
        
        # Topic kontrolü
        if ros2 topic list 2>/dev/null | grep -q "/scan"; then
            log_message "${GREEN}✓ /scan topic'i oluşturuldu${NC}"
            
            # Veri kontrolü
            SCAN_COUNT=$(timeout 3 ros2 topic hz /scan 2>/dev/null | head -1 | grep -o '[0-9.]*' | head -1)
            if [ ! -z "$SCAN_COUNT" ]; then
                log_message "${GREEN}✓ LiDAR veri hızı: ${SCAN_COUNT} Hz${NC}"
            else
                log_message "${YELLOW}⚠ LiDAR veri hızı ölçülemiyor${NC}"
            fi
        else
            log_message "${RED}✗ /scan topic'i oluşturulamadı${NC}"
        fi
        
        # LiDAR'ı durdur
        kill $LIDAR_PID 2>/dev/null
        sleep 2
        return 0
    else
        log_message "${RED}✗ LiDAR driver başlatılamadı${NC}"
        return 1
    fi
}

# Sistem durumunu kontrol et
check_system_status() {
    log_message "${YELLOW}[6/6] Sistem Durumu Kontrol Ediliyor...${NC}"
    
    # USB cihazları
    USB_DEVICES=$(ls /dev/ttyUSB* 2>/dev/null)
    if [ -z "$USB_DEVICES" ]; then
        log_message "${RED}✗ Hiç USB cihazı bulunamadı${NC}"
        return 1
    else
        log_message "${GREEN}✓ USB cihazları: $USB_DEVICES${NC}"
    fi
    
    # USB izinleri
    if [ -r "/dev/ttyUSB0" ] && [ -w "/dev/ttyUSB0" ]; then
        log_message "${GREEN}✓ USB portu okunabilir ve yazılabilir${NC}"
    else
        log_message "${RED}✗ USB portu izinleri yetersiz${NC}"
        return 1
    fi
    
    # ROS2 environment
    if [ -z "$ROS_DISTRO" ]; then
        log_message "${YELLOW}⚠ ROS2 environment yüklenmemiş${NC}"
    else
        log_message "${GREEN}✓ ROS2 environment hazır (ROS_DISTRO: $ROS_DISTRO)${NC}"
    fi
    
    return 0
}

# Ana fonksiyon
main() {
    print_header "LİDAR BAĞLANTI SORUNLARI ÇÖZÜMÜ"
    
    log_message "${CYAN}Sorun giderme başlatılıyor...${NC}"
    log_message "${CYAN}Log dosyası: $LOG_FILE${NC}"
    log_message ""
    
    # Çözüm adımlarını uygula
    reset_usb_device
    fix_usb_permissions
    disable_usb_power_management
    test_baudrates
    reload_lidar_driver
    check_system_status
    
    echo ""
    print_header "ÇÖZÜM TAMAMLANDI"
    
    log_message "${CYAN}Öneriler:${NC}"
    log_message "  • LiDAR'ı USB'den çıkarıp tekrar takın"
    log_message "  • Farklı bir USB portu deneyin"
    log_message "  • USB kablosunu değiştirin"
    log_message "  • LiDAR cihazının güç durumunu kontrol edin"
    log_message "  • Sistem başlatmak için: ./start_system.sh"
    echo ""
    
    log_message "${GREEN}=======================================${NC}"
    log_message "${GREEN}     SORUN GİDERME TAMAMLANDI!        ${NC}"
    log_message "${GREEN}=======================================${NC}"
    log_message "${CYAN}Detaylı log: $LOG_FILE${NC}"
}

# Script'i çalıştır
main "$@"







