#!/bin/bash

# =============================================================================
# LİDAR SİSTEM YÖNETİCİSİ
# =============================================================================
# Bu script LiDAR sisteminin tüm işlemlerini yönetir
# =============================================================================

# Renk kodları
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Başlık yazdırma fonksiyonu
print_header() {
    echo -e "${BLUE}=======================================${NC}"
    echo -e "${BLUE}     $1${NC}"
    echo -e "${BLUE}=======================================${NC}"
    echo ""
}

# Menü göster
show_menu() {
    print_header "LİDAR SİSTEM YÖNETİCİSİ"
    
    echo -e "${CYAN}Lütfen bir seçenek seçin:${NC}"
    echo ""
    echo -e "${GREEN}1.${NC} Tam Kurulum (İlk kez kullanım)"
    echo -e "${GREEN}2.${NC} Hızlı Başlatma (Mevcut kurulum)"
    echo -e "${GREEN}3.${NC} Sistem Durumu Kontrolü"
    echo -e "${GREEN}4.${NC} LiDAR Testi"
    echo -e "${GREEN}5.${NC} Sistem Durdurma"
    echo -e "${GREEN}6.${NC} Log Görüntüleme"
    echo -e "${GREEN}7.${NC} Yardım"
    echo -e "${GREEN}8.${NC} Çıkış"
    echo ""
    echo -n -e "${YELLOW}Seçiminiz (1-8): ${NC}"
}

# Tam kurulum
full_setup() {
    print_header "TAM KURULUM BAŞLATILIYOR"
    echo -e "${YELLOW}Bu işlem sisteminizi tamamen yapılandıracak...${NC}"
    echo -n -e "${YELLOW}Devam etmek istiyor musunuz? (y/N): ${NC}"
    read -r response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        ./setup_lidar_system.sh
    else
        echo -e "${YELLOW}Kurulum iptal edildi.${NC}"
    fi
}

# Hızlı başlatma
quick_start() {
    print_header "HIZLI BAŞLATMA"
    echo -e "${YELLOW}Sistem başlatılıyor...${NC}"
    ./start_system.sh
}

# Sistem durumu kontrolü
system_check() {
    print_header "SİSTEM DURUMU KONTROLÜ"
    ./check_lidar_system.sh
}

# LiDAR testi
lidar_test() {
    print_header "LİDAR TESTİ"
    echo -e "${YELLOW}LiDAR cihazı test ediliyor...${NC}"
    ./test_lidar.sh
}

# Sistem durdurma
stop_system() {
    print_header "SİSTEM DURDURMA"
    echo -e "${YELLOW}Tüm LiDAR süreçleri durduruluyor...${NC}"
    killall -q ros2 rviz2 ydlidar_ros2_driver_node rf2o_laser_odometry_node async_slam_toolbox_node static_transform_publisher
    sleep 2
    echo -e "${GREEN}✓ Tüm süreçler durduruldu.${NC}"
}

# Log görüntüleme
show_logs() {
    print_header "LOG GÖRÜNTÜLEME"
    
    echo -e "${CYAN}Mevcut log dosyaları:${NC}"
    echo ""
    
    # Log dosyalarını listele
    LOG_FILES=($(ls -t /tmp/lidar_*.log /tmp/robot_*.log 2>/dev/null | head -5))
    
    if [ ${#LOG_FILES[@]} -eq 0 ]; then
        echo -e "${YELLOW}Hiç log dosyası bulunamadı.${NC}"
        return
    fi
    
    for i in "${!LOG_FILES[@]}"; do
        echo -e "${GREEN}$((i+1)).${NC} ${LOG_FILES[$i]}"
    done
    
    echo ""
    echo -n -e "${YELLOW}Görüntülemek istediğiniz log dosyasının numarasını girin (1-${#LOG_FILES[@]}): ${NC}"
    read -r choice
    
    if [[ "$choice" =~ ^[0-9]+$ ]] && [ "$choice" -ge 1 ] && [ "$choice" -le ${#LOG_FILES[@]} ]; then
        selected_log="${LOG_FILES[$((choice-1))]}"
        echo -e "${YELLOW}Log dosyası görüntüleniyor: $selected_log${NC}"
        echo -e "${YELLOW}Çıkmak için Ctrl+C tuşlayın${NC}"
        echo ""
        tail -f "$selected_log"
    else
        echo -e "${RED}Geçersiz seçim.${NC}"
    fi
}

# Yardım
show_help() {
    print_header "YARDIM"
    
    echo -e "${CYAN}LiDAR Sistemi Kullanım Kılavuzu:${NC}"
    echo ""
    echo -e "${GREEN}1. Tam Kurulum:${NC}"
    echo "   • İlk kez kullanım için"
    echo "   • Tüm gerekli paketleri kurar"
    echo "   • Konfigürasyon dosyalarını oluşturur"
    echo "   • USB izinlerini ayarlar"
    echo ""
    echo -e "${GREEN}2. Hızlı Başlatma:${NC}"
    echo "   • Mevcut kurulumu kullanır"
    echo "   • Sistemi hızlıca başlatır"
    echo "   • Günlük kullanım için"
    echo ""
    echo -e "${GREEN}3. Sistem Durumu Kontrolü:${NC}"
    echo "   • Sistem durumunu kontrol eder"
    echo "   • Sorunları tespit eder"
    echo "   • Öneriler sunar"
    echo ""
    echo -e "${GREEN}4. LiDAR Testi:${NC}"
    echo "   • LiDAR cihazını test eder"
    echo "   • Farklı ayarları dener"
    echo "   • En iyi konfigürasyonu bulur"
    echo ""
    echo -e "${GREEN}5. Sistem Durdurma:${NC}"
    echo "   • Tüm süreçleri durdurur"
    echo "   • Temizlik yapar"
    echo ""
    echo -e "${GREEN}6. Log Görüntüleme:${NC}"
    echo "   • Log dosyalarını listeler"
    echo "   • Canlı log görüntüleme"
    echo ""
    echo -e "${CYAN}Dosya Yapısı:${NC}"
    echo "  • setup_lidar_system.sh    - Tam kurulum"
    echo "  • start_system.sh          - Hızlı başlatma"
    echo "  • check_lidar_system.sh    - Durum kontrolü"
    echo "  • test_lidar.sh            - LiDAR testi"
    echo "  • config/                  - Konfigürasyon dosyaları"
    echo "  • README_LIDAR_SYSTEM.md   - Detaylı kılavuz"
    echo ""
    echo -e "${CYAN}Sorun Giderme:${NC}"
    echo "  • USB bağlantısını kontrol edin"
    echo "  • USB izinlerini kontrol edin"
    echo "  • ROS2 environment'ı kontrol edin"
    echo "  • Log dosyalarını inceleyin"
    echo ""
}

# Ana döngü
main() {
    while true; do
        clear
        show_menu
        read -r choice
        
        case $choice in
            1)
                full_setup
                ;;
            2)
                quick_start
                ;;
            3)
                system_check
                ;;
            4)
                lidar_test
                ;;
            5)
                stop_system
                ;;
            6)
                show_logs
                ;;
            7)
                show_help
                ;;
            8)
                print_header "ÇIKIŞ"
                echo -e "${GREEN}Güle güle!${NC}"
                exit 0
                ;;
            *)
                echo -e "${RED}Geçersiz seçim. Lütfen 1-8 arasında bir sayı girin.${NC}"
                ;;
        esac
        
        echo ""
        echo -n -e "${YELLOW}Devam etmek için Enter tuşuna basın...${NC}"
        read -r
    done
}

# Script'i çalıştır
main "$@"







