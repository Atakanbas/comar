#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <uORB/topics/input_rc.h>
#include <parameters/param.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>
#include <stm32_gpio.h>

extern "C" __EXPORT int dir_out_main(int argc, char *argv[]);

// sabitler
static constexpr uint16_t PWM_STOP   = 800;   // senin motorun durduğu değer
static constexpr uint16_t PWM_MAXSPD = 2000;
static constexpr uint16_t RC_MID     = 1500;
static constexpr uint16_t RC_DB      = 50;    // RC5 için ±50
static constexpr uint16_t STEER_DB   = 100;   // RC1 (steer) için ±100

// GPIO pin tanımları
// AUX3 GPIO pini: PH11 (ileri/geri yön)
#define DIR_GPIO      (GPIO_PORTH | GPIO_PIN11)
// AUX1 GPIO pini: PI0 (güç - dönüş var mı)
#define DIR_FLAG_PIN  (GPIO_PORTI | GPIO_PIN0)
// AUX4 GPIO pini: PH10 (sağ/sol yön)
#define DIR_PWR_PIN   (GPIO_PORTH | GPIO_PIN10)
// motor = MAIN6 (0-based index = 5)
#define MOTOR_CH      6   // MAIN6 (1-based)

int dir_out_main(int, char *[])
{
    PX4_INFO("DIR_OUT: start (RC1=steer, RC2=hiz, RC5=vites, MAIN6=motor, AUX1=power, AUX3=direction, AUX4=steer_dir)");

    // RC maplerini oku
    int32_t rc_map_roll     = 1;   // RC1 (steer/sağ-sol)
    int32_t rc_map_throttle = 2;   // RC2 (hız)
    int32_t rc_map_aux1     = 5;   // RC5 (vites/yön)
    (void)param_get(param_find("RC_MAP_ROLL"),     &rc_map_roll);
    (void)param_get(param_find("RC_MAP_THROTTLE"), &rc_map_throttle);
    (void)param_get(param_find("RC_MAP_AUX1"),     &rc_map_aux1);

    int steer_idx = rc_map_roll     - 1;
    int thr_idx   = rc_map_throttle - 1;
    int gear_idx  = rc_map_aux1     - 1;

    // MAIN6 pwm ayaklarini hazirla
    // MAIN indexi C API'de 0-bazli, ama sen 6. çıkışı kullanıyorsun -> index 5
    int motor_index = 5;
    uint32_t motor_mask = (1u << motor_index);
    if (up_pwm_servo_init(motor_mask) < 0) {
        PX4_ERR("DIR_OUT: MAIN6 PWM init fail");
        return -1;
    }
    up_pwm_servo_arm(true, motor_mask);

    // başlangıçta dur
    up_pwm_servo_set(motor_index, PWM_STOP);
    up_pwm_update(motor_mask);

    // AUX3'ü GPIO olarak yapılandır (PWM değil)
    // ÖNEMLİ: QGC'de AUX3'ü Disabled yapman gerekiyor, yoksa PWM mixer çakışır
    px4_arch_configgpio(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | DIR_GPIO);

    // AUX1 ve AUX4'ü GPIO olarak yapılandır (sağ-sol kontrolü için)
    // ÖNEMLİ: QGC'de AUX1 ve AUX4'ü de Disabled yapman gerekiyor
    px4_arch_configgpio(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | DIR_FLAG_PIN);
    px4_arch_configgpio(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | DIR_PWR_PIN);

    PX4_INFO("DIR_OUT: PWM initialized - MAIN6(ch=%d), GPIO: AUX1=power, AUX3=direction, AUX4=steer_dir", motor_index);

    // RC subscribe
    int rc_sub = orb_subscribe(ORB_ID(input_rc));
    px4_pollfd_struct_t fds[1]{};
    fds[0].fd = rc_sub;
    fds[0].events = POLLIN;

    PX4_INFO("DIR_OUT: loop ready");

    while (true) {
        if (px4_poll(fds, 1, 100) <= 0) {
            continue;
        }

        input_rc_s rc{};
        orb_copy(ORB_ID(input_rc), rc_sub, &rc);
        if (steer_idx >= (int)rc.channel_count) continue;
        if (thr_idx   >= (int)rc.channel_count) continue;
        if (gear_idx  >= (int)rc.channel_count) continue;

        uint16_t rc_steer = rc.values[steer_idx]; // RC1 (sağ-sol)
        uint16_t rc_thr   = rc.values[thr_idx];   // RC2 (hız)
        uint16_t rc_gear  = rc.values[gear_idx];  // RC5 (vites/yön)

        // 1) vitesi belirle
        enum {GEAR_NEUTRAL=0, GEAR_FWD=1, GEAR_REV=2};
        int gear = GEAR_NEUTRAL;
        if (rc_gear < (RC_MID - RC_DB)) {
            gear = GEAR_FWD;    // ileri
        } else if (rc_gear > (RC_MID + RC_DB)) {
            gear = GEAR_REV;    // geri
        } else {
            gear = GEAR_NEUTRAL; // boş
        }

        // 2) hiz hesapla
        uint16_t motor_pwm = PWM_STOP;
        if (gear == GEAR_NEUTRAL) {
            motor_pwm = PWM_STOP;
        } else {
            if (rc_thr <= 1000) {
                motor_pwm = PWM_STOP;
            } else {
                // 1000..2000 → 0..1
                float norm = (rc_thr - 1000) / 1000.0f; // 0..1
                if (norm < 0.f) norm = 0.f;
                if (norm > 1.f) norm = 1.f;
                motor_pwm = (uint16_t)(PWM_STOP + norm * (PWM_MAXSPD - PWM_STOP));
            }
        }

        // 3) motor PWM'ini yaz (MAIN6)
        up_pwm_servo_set(motor_index, motor_pwm);
        up_pwm_update(motor_mask);

        // 4) direction'i AUX3'e yaz (GPIO - dijital seviye):
        // İleri: GPIO LOW (0) → 0V
        // Geri: GPIO HIGH (1) → 3.3V
        // Nötr: GPIO LOW (0) → 0V
        bool dir_level = false; // default LOW (ileri/nötr)
        if (gear == GEAR_FWD) {
            dir_level = false;  // ileri → 0V
        } else if (gear == GEAR_REV) {
            dir_level = true;   // geri → 3.3V
        } else {
            // NÖTR: 0V
            dir_level = false;
        }
        px4_arch_gpiowrite(DIR_GPIO, dir_level);

        // 5) Steer kontrolü (sağ-sol) - AUX1 ve AUX4
        int32_t steer_diff = (int32_t)rc_steer - RC_MID;
        int32_t abs_steer_diff = (steer_diff < 0) ? -steer_diff : steer_diff;
        const char* steer_dir = "ORTADA";
        int steer_magnitude = 0;

        // Hareket var mı? (deadband dışında)
        bool direction_active = false;
        bool power_right = false;

        if (abs_steer_diff > STEER_DB) {
            // Deadband dışında: hareket var
            direction_active = true;

            // Sağa mı sola mı?
            if (steer_diff > 0) {
                power_right = true;  // Sağa gidiyor
                steer_dir = "SAG";
                steer_magnitude = (int)((abs_steer_diff * 100) / 500); // 0-100%
            } else {
                power_right = false; // Sola gidiyor
                steer_dir = "SOL";
                steer_magnitude = (int)((abs_steer_diff * 100) / 500); // 0-100%
            }
        } else {
            // Ortada: hareket yok
            direction_active = false;
            power_right = false;
        }

        // AUX1 (Movement flag): Hareket var mı? (ters mantık)
        px4_arch_gpiowrite(DIR_FLAG_PIN, !direction_active);

        // AUX4 (Power): Sağa mı? (ters mantık)
        px4_arch_gpiowrite(DIR_PWR_PIN, !power_right);

        // 6) Yön bilgisi (ileri/geri)
        const char* gear_str = "NÖTR";
        if (gear == GEAR_FWD) {
            gear_str = "ILERI";
        } else if (gear == GEAR_REV) {
            gear_str = "GERI";
        }

        // Kısa loglama (her döngüde)
        PX4_INFO("RC1=%u RC2=%u RC5=%u | Y=%s S=%s(%d%%) | M=%u AUX1=%d AUX3=%s AUX4=%d",
                 rc_steer, rc_thr, rc_gear,
                 gear_str, steer_dir, steer_magnitude,
                 motor_pwm,
                 (int)!direction_active,  // AUX1 (ters mantık)
                 dir_level ? "HIGH(3.3V)" : "LOW(0V)",  // AUX3
                 (int)!power_right);  // AUX4 (ters mantık)
    }

    // Cleanup
    up_pwm_servo_arm(false, motor_mask);
    up_pwm_servo_deinit(motor_mask);
    return 0;
}
