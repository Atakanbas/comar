#!/bin/bash

# =============================================================================
# LİDAR TEST SCRIPTİ
# =============================================================================
# Bu script LiDAR cihazını test eder ve sorunları tespit eder
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
LOG_FILE="/tmp/lidar_test_$(date +%Y%m%d_%H%M%S).log"

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

# USB portunu test et
test_usb_port() {
    log_message "${YELLOW}[1/6] USB Port Testi${NC}"
    
    USB_DEVICES=$(ls /dev/ttyUSB* 2>/dev/null)
    if [ -z "$USB_DEVICES" ]; then
        log_message "${RED}✗ Hiç USB cihazı bulunamadı!${NC}"
        return 1
    fi
    
    for device in $USB_DEVICES; do
        if lsof "$device" > /dev/null 2>&1; then
            log_message "${YELLOW}⚠ $device portu meşgul${NC}"
        else
            log_message "${GREEN}✓ $device portu kullanılabilir${NC}"
            
            # Port izinlerini kontrol et
            if [ -r "$device" ] && [ -w "$device" ]; then
                log_message "${GREEN}✓ $device portu okunabilir ve yazılabilir${NC}"
            else
                log_message "${RED}✗ $device portu izinleri yetersiz${NC}"
                log_message "${YELLOW}Çözüm: sudo chmod 666 $device${NC}"
            fi
        fi
    done
    
    return 0
}

# LiDAR driver'ı test et
test_lidar_driver() {
    log_message "${YELLOW}[2/6] LiDAR Driver Testi${NC}"
    
    # ROS2 environment'ı yükle
    source /opt/ros/humble/setup.bash 2>/dev/null
    source /home/enelsis-pc/enelsis_moab_ws/install/setup.bash 2>/dev/null
    
    # LiDAR driver'ı başlat
    log_message "${YELLOW}LiDAR driver başlatılıyor...${NC}"
    timeout 10 ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node \
        --ros-args --params-file config/lidar_stable.yaml &
    LIDAR_PID=$!
    
    sleep 8
    
    # LiDAR çalışıyor mu kontrol et
    if kill -0 $LIDAR_PID 2>/dev/null; then
        log_message "${GREEN}✓ LiDAR driver başlatıldı (PID: $LIDAR_PID)${NC}"
        
        # Topic kontrolü
        if ros2 topic list 2>/dev/null | grep -q "/scan"; then
            log_message "${GREEN}✓ /scan topic'i oluşturuldu${NC}"
            
            # Veri kontrolü
            SCAN_COUNT=$(timeout 5 ros2 topic hz /scan 2>/dev/null | head -1 | grep -o '[0-9.]*' | head -1)
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

# Farklı baudrate'leri test et
test_baudrates() {
    log_message "${YELLOW}[3/6] Baudrate Testi${NC}"
    
    BAUDRATES=(115200 128000 230400 460800)
    WORKING_BAUDRATES=()
    
    for baudrate in "${BAUDRATES[@]}"; do
        log_message "${YELLOW}Testing baudrate: $baudrate${NC}"
        
        # Konfigürasyonu güncelle
        sed -i "s/baudrate: [0-9]*/baudrate: $baudrate/" config/lidar_stable.yaml
        
        # LiDAR'ı test et
        timeout 5 ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node \
            --ros-args --params-file config/lidar_stable.yaml > /dev/null 2>&1 &
        TEST_PID=$!
        
        sleep 3
        
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

# Farklı frekansları test et
test_frequencies() {
    log_message "${YELLOW}[4/6] Frekans Testi${NC}"
    
    FREQUENCIES=(5.0 7.0 10.0 15.0)
    WORKING_FREQUENCIES=()
    
    for freq in "${FREQUENCIES[@]}"; do
        log_message "${YELLOW}Testing frequency: $freq Hz${NC}"
        
        # Konfigürasyonu güncelle
        sed -i "s/frequency: [0-9.]*/frequency: $freq/" config/lidar_stable.yaml
        
        # LiDAR'ı test et
        timeout 5 ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node \
            --ros-args --params-file config/lidar_stable.yaml > /dev/null 2>&1 &
        TEST_PID=$!
        
        sleep 3
        
        if kill -0 $TEST_PID 2>/dev/null; then
            log_message "${GREEN}✓ Frekans $freq Hz çalışıyor${NC}"
            WORKING_FREQUENCIES+=($freq)
        else
            log_message "${RED}✗ Frekans $freq Hz çalışmıyor${NC}"
        fi
        
        kill $TEST_PID 2>/dev/null
        sleep 1
    done
    
    if [ ${#WORKING_FREQUENCIES[@]} -gt 0 ]; then
        log_message "${GREEN}✓ Çalışan frekanslar: ${WORKING_FREQUENCIES[*]}${NC}"
        # En iyi frekansı seç
        BEST_FREQ=${WORKING_FREQUENCIES[0]}
        sed -i "s/frequency: [0-9.]*/frequency: $BEST_FREQ/" config/lidar_stable.yaml
        log_message "${GREEN}✓ En iyi frekans seçildi: $BEST_FREQ Hz${NC}"
        return 0
    else
        log_message "${RED}✗ Hiçbir frekans çalışmıyor${NC}"
        return 1
    fi
}

# USB güç yönetimini test et
test_usb_power() {
    log_message "${YELLOW}[5/6] USB Güç Yönetimi Testi${NC}"
    
    if [ -f "/sys/bus/usb-serial/devices/ttyUSB0/power/control" ]; then
        CURRENT_POWER=$(cat /sys/bus/usb-serial/devices/ttyUSB0/power/control 2>/dev/null)
        log_message "${CYAN}Mevcut güç yönetimi: $CURRENT_POWER${NC}"
        
        if [ "$CURRENT_POWER" = "auto" ]; then
            log_message "${YELLOW}Güç yönetimi 'on' olarak ayarlanıyor...${NC}"
            echo 'on' | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/power/control > /dev/null
            log_message "${GREEN}✓ Güç yönetimi 'on' olarak ayarlandı${NC}"
        else
            log_message "${GREEN}✓ Güç yönetimi zaten 'on'${NC}"
        fi
    else
        log_message "${YELLOW}⚠ USB güç yönetimi dosyası bulunamadı${NC}"
    fi
    
    return 0
}

# Sistem performansını test et
test_system_performance() {
    log_message "${YELLOW}[6/6] Sistem Performansı Testi${NC}"
    
    # CPU kullanımı
    CPU_USAGE=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | awk -F'%' '{print $1}')
    log_message "${CYAN}CPU kullanımı: ${CPU_USAGE}%${NC}"
    
    # Bellek kullanımı
    MEMORY_USAGE=$(free | grep Mem | awk '{printf "%.1f", $3/$2 * 100.0}')
    log_message "${CYAN}Bellek kullanımı: ${MEMORY_USAGE}%${NC}"
    
    # USB cihazı bilgileri
    if command -v lsusb > /dev/null; then
        log_message "${CYAN}USB cihazları:${NC}"
        lsusb | grep -i "serial\|usb\|tty" | sed 's/^/  /'
    fi
    
    # USB cihazı detayları
    if [ -f "/sys/bus/usb-serial/devices/ttyUSB0/uevent" ]; then
        log_message "${CYAN}USB cihazı detayları:${NC}"
        cat /sys/bus/usb-serial/devices/ttyUSB0/uevent | sed 's/^/  /'
    fi
    
    return 0
}

# Ana fonksiyon
main() {
    print_header "LİDAR TEST SCRIPTİ"
    
    log_message "${CYAN}Test başlatılıyor...${NC}"
    log_message "${CYAN}Log dosyası: $LOG_FILE${NC}"
    log_message ""
    
    # Testleri yap
    USB_OK=$(test_usb_port && echo "true" || echo "false")
    DRIVER_OK=$(test_lidar_driver && echo "true" || echo "false")
    BAUDRATE_OK=$(test_baudrates && echo "true" || echo "false")
    FREQ_OK=$(test_frequencies && echo "true" || echo "false")
    POWER_OK=$(test_usb_power && echo "true" || echo "false")
    PERF_OK=$(test_system_performance && echo "true" || echo "false")
    
    echo ""
    print_header "TEST SONUÇLARI"
    
    # Sonuçları özetle
    log_message "${CYAN}Test Sonuçları:${NC}"
    log_message "  • USB Port: $([ "$USB_OK" = "true" ] && echo "✓" || echo "✗")"
    log_message "  • LiDAR Driver: $([ "$DRIVER_OK" = "true" ] && echo "✓" || echo "✗")"
    log_message "  • Baudrate: $([ "$BAUDRATE_OK" = "true" ] && echo "✓" || echo "✗")"
    log_message "  • Frekans: $([ "$FREQ_OK" = "true" ] && echo "✓" || echo "✗")"
    log_message "  • USB Güç: $([ "$POWER_OK" = "true" ] && echo "✓" || echo "✗")"
    log_message "  • Performans: $([ "$PERF_OK" = "true" ] && echo "✓" || echo "✗")"
    echo ""
    
    # Genel durum
    if [ "$USB_OK" = "true" ] && [ "$DRIVER_OK" = "true" ] && [ "$BAUDRATE_OK" = "true" ] && [ "$FREQ_OK" = "true" ]; then
        log_message "${GREEN}=======================================${NC}"
        log_message "${GREEN}     LİDAR TESTİ BAŞARILI!            ${NC}"
        log_message "${GREEN}=======================================${NC}"
        log_message "${GREEN}LiDAR cihazı çalışıyor ve yapılandırıldı${NC}"
    else
        log_message "${RED}=======================================${NC}"
        log_message "${RED}     LİDAR TESTİ BAŞARISIZ!            ${NC}"
        log_message "${RED}=======================================${NC}"
        log_message "${RED}Lütfen LiDAR bağlantısını ve konfigürasyonunu kontrol edin${NC}"
    fi
    
    echo ""
    log_message "${CYAN}Öneriler:${NC}"
    if [ "$USB_OK" = "false" ]; then
        log_message "  • LiDAR'ı USB'ye bağlayın"
        log_message "  • USB kablosunu değiştirin"
        log_message "  • USB portunu değiştirin"
    fi
    if [ "$DRIVER_OK" = "false" ]; then
        log_message "  • LiDAR driver'ını yeniden yükleyin"
        log_message "  • ROS2 environment'ı kontrol edin"
    fi
    if [ "$BAUDRATE_OK" = "false" ]; then
        log_message "  • LiDAR cihazının baudrate'ini kontrol edin"
        log_message "  • Manuel olarak baudrate ayarlayın"
    fi
    
    log_message "${BLUE}=======================================${NC}"
    log_message "${BLUE}         TEST TAMAMLANDI               ${NC}"
    log_message "${BLUE}=======================================${NC}"
    log_message "${CYAN}Detaylı log: $LOG_FILE${NC}"
}

# Script'i çalıştır
main "$@"







