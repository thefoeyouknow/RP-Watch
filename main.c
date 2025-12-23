/**
 * Waveshare RP2040-Touch-LCD-1.28
 * FIRMWARE: v1.0.1-RELEASE
 * STATUS: Stable Release - Power Optimized
 * CHANGES: 
 * - Step counter persistence across tile transitions
 * - Comprehensive power management optimization
 * - ADC and IMU power state management
 * - Activity screen redesign with 300° progress arc
 * - Watchdog stability improvements
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h> 
#include <malloc.h> 

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/rtc.h"
#include "hardware/adc.h"
#include "hardware/regs/adc.h" // For ADC register access
#include "hardware/pwm.h" 
#include "hardware/watchdog.h" 
#include "hardware/clocks.h"
#include "hardware/timer.h" 
#include "hardware/flash.h" 
#include "hardware/sync.h"
#include "pico/util/datetime.h"
#include "pico/bootrom.h"
#include "pico/stdio_usb.h" 

#include "lvgl.h" 
#include "lv_conf.h" 

// ==========================================
//           CONFIGURATION
// ==========================================
#define FIRMWARE_VERSION "v1.0.1-RELEASE"
#define LCD_HOR_RES      240
#define LCD_VER_RES      240
#define DRAW_BUFFER_SIZE (240 * 40) // Reduced buffer for power savings 

// Hardware Pins
#define LCD_RST_PIN  13
#define LCD_DC_PIN   8
#define LCD_CS_PIN   9
#define LCD_BL_PIN   25 
#define LCD_SPI_PORT spi1
#define LCD_CLK_PIN  10
#define LCD_MOSI_PIN 11

#define I2C_PORT       i2c1
#define I2C_SDA_PIN    6
#define I2C_SCL_PIN    7
#define TOUCH_ADDR     0x15
#define TOUCH_RST_PIN  22 
#define BATTERY_PIN    29 
#define BUZZER_PIN     28

// Settings
#define TIMEOUT_IDLE_MS  10000 
#define TIMEOUT_SLEEP_MS 30000 
#define LIFT_ARM_THRES   1500    
#define LIFT_WAKE_THRES  3000    
#define STEP_THRESHOLD_BASE 8000 
#define STEP_DEBOUNCE_MS    300
#define STEP_GOAL_DEFAULT   10000 
#define FLASH_TARGET_OFFSET (2 * 1024 * 1024 - 4096) 
#define SETTINGS_MAGIC      0xFEEDC0DE 
#define PI 3.1415926535f 

// Step detection filter parameters
#define STEP_HP_ALPHA         0.20f   // High-pass filter coefficient
#define STEP_HP_BASE          1500.0f // Base high-pass threshold
#define STEP_HP_MIN           900.0f  // Minimum threshold floor
#define STEP_HYST_LOW_FACTOR  0.60f   // Hysteresis: low = high * factor

enum { ALARM_MODE_ONCE, ALARM_MODE_DAILY, ALARM_MODE_WEEKDAY, ALARM_MODE_WEEKEND };
enum { TMR_SETUP, TMR_RUNNING, TMR_PAUSED, TMR_FIRING };
typedef enum { PWR_ACTIVE, PWR_IDLE, PWR_SLEEP } PowerState;

typedef struct {
    uint32_t magic;         
    uint8_t  brightness;    
    uint8_t  imu_sens;      
    uint8_t  touch_sens;    
    uint32_t step_goal;     
    uint32_t total_lifetime_steps;   
    uint32_t steps_today;   
    uint8_t  last_day_saved;
    uint8_t  lift_sens; 
    uint32_t step_history[7];
    uint8_t alarm_h;    
    uint8_t alarm_m;    
    uint8_t alarm_mode; 
    bool    alarm_enabled; 
    uint8_t timer_h;
    uint8_t timer_m;
    uint8_t timer_s;
} watch_settings_t;

// --- PROTOTYPES ---
void clean_pilot_globals(void);
void clean_tools_globals(void);
void clean_activity_globals(void);
void clean_timeset_globals(void);
void imu_set_low_power_mode(bool enable);
void update_step_counter(void);
void build_timer_page(lv_obj_t *parent);
void build_alarm_page(lv_obj_t *parent);
uint32_t session_steps = 0; 
uint32_t last_step_time = 0;
static uint32_t last_raw_update = 0;
static uint32_t last_imu_read_sleep = 0; // For sleep mode IMU throttling
int16_t acc_x, acc_y, acc_z; 

// Timer State
uint8_t timer_state = TMR_SETUP;
uint32_t timer_duration_sec = 0;
uint32_t timer_remaining_sec = 0;
uint32_t timer_last_tick = 0;

// Alarm State
bool alarm_firing = false;
uint32_t alarm_last_beep = 0;
bool alarm_beep_state = false;

// Clock Sync
static int8_t prev_sec = -1;
static uint32_t last_sec_time = 0;

// Performance metrics
uint32_t loop_start_time = 0;
uint32_t cpu_load_pct = 0;

// Global State
watch_settings_t settings;               // Persistent settings stored in flash
PowerState pwr_state = PWR_ACTIVE;       // Current power state
datetime_t t_now;                        // Current RTC time
uint32_t last_activity_time = 0;         // Last time of any user activity
bool is_date_synced = false;             // USB time sync status

// Battery telemetry
float battery_voltage = 0.0f;
uint8_t battery_pct = 0;
bool is_charging = false;
uint32_t last_bat_read = 0;
static float battery_avg = 0.0f;

// Touch/IMU detection state
bool touch_found = false;
bool imu_found = false;
uint8_t detected_imu_addr = 0;

// LVGL draw buffer
lv_disp_draw_buf_t disp_buf;
static lv_color_t buf_1[DRAW_BUFFER_SIZE];

// Buzzer state
static uint32_t tone_end_time = 0;
static bool tone_playing = false;

// Lift-to-wake state
bool lift_trigger_armed = false;
uint32_t lift_arm_time = 0;

// Settings save debounce
static uint32_t settings_changed_time = 0;
static bool settings_needs_save = false;

// Tile Objects
lv_obj_t *tv; 
lv_obj_t *tile_center = NULL; 
lv_obj_t *tile_left = NULL;   
lv_obj_t *tile_right = NULL;  
lv_obj_t *tile_top = NULL;    
lv_obj_t *tile_bottom = NULL; 
lv_obj_t *g_sub_scr = NULL; 

// UI Globals - Pilot
lv_obj_t *g_batt_cont = NULL; 
lv_obj_t *g_batt_bars[5];     
lv_obj_t *g_step_arc = NULL;     
lv_obj_t *g_step_label = NULL;
lv_obj_t *g_time_label = NULL;
lv_obj_t *g_line_hour = NULL;
lv_obj_t *g_line_min = NULL;
lv_obj_t *g_line_sec = NULL;
lv_obj_t *g_day_label = NULL;  
lv_obj_t *g_month_label = NULL;
lv_obj_t *g_date_box = NULL;   
lv_obj_t *g_date_num = NULL;   

// UI Globals - Activity
lv_obj_t *g_activity_arc = NULL;
lv_obj_t *g_activity_steps_label = NULL;
lv_obj_t *g_activity_goal_label = NULL;
lv_obj_t *g_activity_pct_label = NULL;

// UI Globals - Tools
lv_obj_t *g_diag_label = NULL; 

// UI Globals - Clock Set
lv_obj_t *g_roller_hour = NULL;
lv_obj_t *g_roller_min = NULL;

// Stopwatch Globals (Digital)
bool sw_running = false;
uint32_t sw_start_time = 0;
uint32_t sw_accumulated_time = 0;
lv_obj_t *g_sw_label_time = NULL;
lv_obj_t *g_sw_btn_toggle = NULL;
lv_obj_t *g_sw_label_toggle = NULL;
lv_obj_t *g_sw_label_laps = NULL; 
static char sw_lap_buffer[512] = "";

// Timer Globals  
lv_obj_t *g_tmr_modal = NULL;
lv_obj_t *g_alarm_modal = NULL;
lv_obj_t *g_tmr_cont_setup = NULL; lv_obj_t *g_tmr_cont_run = NULL;
lv_obj_t *g_tmr_roller_h = NULL; lv_obj_t *g_tmr_roller_m = NULL; lv_obj_t *g_tmr_roller_s = NULL;
lv_obj_t *g_tmr_arc = NULL; lv_obj_t *g_tmr_lbl_digits = NULL;
lv_obj_t *g_tmr_btn_start = NULL; lv_obj_t *g_tmr_btn_pause = NULL;

// Alarm Globals
lv_obj_t *g_alarm_roller_h = NULL; lv_obj_t *g_alarm_roller_m = NULL;
lv_obj_t *g_alarm_roller_ap = NULL; lv_obj_t *g_alarm_roller_mode = NULL; lv_obj_t *g_alarm_sw = NULL;

// Points for lines
lv_point_t line_points_hour[2];
lv_point_t line_points_min[2];
lv_point_t line_points_sec[2];

const char *DAY_NAMES[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
const char *MONTH_NAMES[] = {"", "JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};
const float LIPO_LUT[11] = { 4.20f, 4.11f, 4.02f, 3.95f, 3.87f, 3.84f, 3.80f, 3.77f, 3.73f, 3.69f, 3.27f };


// --- HARDWARE DRIVERS ---

void set_backlight(uint8_t brightness) {
    uint16_t pwm_val = (uint16_t)brightness * (uint16_t)brightness; 
    pwm_set_gpio_level(LCD_BL_PIN, pwm_val);
}

// BUZZER DRIVER
void init_buzzer(void) {
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    // Set clock divider for better frequency control
    pwm_set_clkdiv(slice_num, 1.0f); // No division for maximum flexibility
    pwm_set_enabled(slice_num, false);
}

void play_tone(uint16_t freq, uint16_t duration_ms) {
    if (freq == 0) {
        pwm_set_gpio_level(BUZZER_PIN, 0);
        return;
    }
    
    // RP2040 Clock is ~133MHz. 
    // wrap = (sys_clk / freq) - 1 for accurate frequency
    // 30% duty cycle often produces louder output on piezo buzzers
    uint32_t clock = 133000000;
    uint32_t wrap = (clock / freq) - 1;
    
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_gpio_level(BUZZER_PIN, wrap * 30 / 100); // 30% Duty Cycle for louder output
    pwm_set_enabled(slice_num, true);
    
    tone_end_time = to_ms_since_boot(get_absolute_time()) + duration_ms;
    tone_playing = true;
}

void check_buzzer_state(uint32_t now) {
    if (tone_playing && now >= tone_end_time) {
        uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
        pwm_set_enabled(slice_num, false);
        tone_playing = false;
    }
}

void set_power_state(PowerState new_state) {
    if (pwr_state == new_state) return;
    uint slice_num = pwm_gpio_to_slice_num(LCD_BL_PIN);
    switch (new_state) {
        case PWR_ACTIVE: 
            set_backlight(settings.brightness);
            pwm_set_enabled(slice_num, true);
            imu_set_low_power_mode(false); // Normal power mode
            break;
        case PWR_IDLE:   
            set_backlight(settings.brightness / 4);
            pwm_set_enabled(slice_num, true);
            imu_set_low_power_mode(true); // Low power mode
            break;
        case PWR_SLEEP:  
            set_backlight(0);
            pwm_set_enabled(slice_num, false); // Disable PWM completely
            imu_set_low_power_mode(true); // Low power mode
            break;
    }
    pwr_state = new_state;
}

void load_settings(void) {
    const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);
    memcpy(&settings, flash_target_contents, sizeof(watch_settings_t));
    if (settings.magic != SETTINGS_MAGIC) {
        settings.magic = SETTINGS_MAGIC;
        settings.brightness = 200;
        settings.imu_sens = 50;
        settings.touch_sens = 50;
        settings.step_goal = STEP_GOAL_DEFAULT;
        settings.total_lifetime_steps = 0;
        settings.steps_today = 0;
        settings.last_day_saved = 0;
        settings.lift_sens = 50; // Initialize lift sensitivity
        for(int i=0;i<7;i++) settings.step_history[i] = 0; 
    }
    set_backlight(settings.brightness);
}

void save_settings(void) {
    settings.total_lifetime_steps += session_steps;
    settings.steps_today += session_steps;
    session_steps = 0; 
    if (t_now.day != settings.last_day_saved && settings.last_day_saved != 0) {
        for(int i=6; i>0; i--) settings.step_history[i] = settings.step_history[i-1]; 
        settings.step_history[0] = settings.steps_today; 
        settings.steps_today = 0; 
        settings.last_day_saved = t_now.day;
    } else {
        settings.last_day_saved = t_now.day;
    }
    
    // Validate settings before writing to prevent corruption
    if (settings.brightness < 10) settings.brightness = 10;
    if (settings.brightness > 255) settings.brightness = 255;
    if (settings.step_goal < 100) settings.step_goal = STEP_GOAL_DEFAULT;
    
    // Update watchdog before flash operation
    watchdog_update();
    
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, (uint8_t *)&settings, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
    
    // Update watchdog after flash operation
    watchdog_update();
}

// LCD Drivers
void lcd_write_cmd(uint8_t cmd) {
    gpio_put(LCD_DC_PIN, 0); gpio_put(LCD_CS_PIN, 0); spi_write_blocking(LCD_SPI_PORT, &cmd, 1); gpio_put(LCD_CS_PIN, 1); 
}
void lcd_write_data(uint8_t data) {
    gpio_put(LCD_DC_PIN, 1); gpio_put(LCD_CS_PIN, 0); spi_write_blocking(LCD_SPI_PORT, &data, 1); gpio_put(LCD_CS_PIN, 1); 
}
void lcd_set_window(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    lcd_write_cmd(0x2A); lcd_write_data(x >> 8); lcd_write_data(x & 0xFF); 
    lcd_write_data((x + w - 1) >> 8); lcd_write_data((x + w - 1) & 0xFF); 
    lcd_write_cmd(0x2B); 
    lcd_write_data(y >> 8); lcd_write_data(y & 0xFF); 
    lcd_write_data((y + h - 1) >> 8); lcd_write_data((y + h - 1) & 0xFF); 
    lcd_write_cmd(0x2C); 
}
void lcd_clear_screen(uint16_t color) {
    lcd_set_window(0, 0, 240, 240); gpio_put(LCD_DC_PIN, 1); gpio_put(LCD_CS_PIN, 0); 
    uint8_t line[480]; for (int i = 0; i < 240; i++) { line[i*2] = color >> 8; line[i*2+1] = color & 0xFF; } 
    for (int y = 0; y < 240; y++) { spi_write_blocking(LCD_SPI_PORT, line, 480); } gpio_put(LCD_CS_PIN, 1); 
}

void lcd_init(void) {
    gpio_put(LCD_RST_PIN, 1); sleep_ms(10); 
    gpio_put(LCD_RST_PIN, 0); sleep_ms(10); 
    gpio_put(LCD_RST_PIN, 1); sleep_ms(50);
    const uint8_t init_cmds[] = { 
        0xEF, 0, 0xEB, 1, 0x14, 0xFE, 0, 0xEF, 0, 
        0xEB, 1, 0x14, 0x84, 1, 0x40, 0x85, 1, 0xFF, 
        0x86, 1, 0xFF, 0x87, 1, 0xFF, 0x88, 1, 0x0A, 
        0x89, 1, 0x21, 0x8A, 1, 0x00, 0x8B, 1, 0x80, 
        0x8C, 1, 0x01, 0x8D, 1, 0x01, 0x8E, 1, 0xFF, 
        0x8F, 1, 0xFF, 0xB6, 2, 0x00, 0x20, 0x36, 1, 
        0x98, 0x3A, 1, 0x05, 0x90, 4, 0x08, 0x08, 0x08, 0x08, 
        0xBD, 1, 0x06, 0xBC, 1, 0x00, 0xFF, 3, 0x60, 0x01, 0x04, 
        0xC3, 1, 0x13, 0xC4, 1, 0x13, 0xC9, 1, 0x22, 0xBE, 1, 0x11, 
        0xE1, 2, 0x10, 0x0E, 0xDF, 3, 0x21, 0x0c, 0x02, 0xF0, 6, 
        0x45, 0x09, 0x08, 0x08, 0x26, 0x2A, 0xF1, 6, 0x43, 0x70, 
        0x72, 0x36, 0x37, 0x6F, 0xF2, 6, 0x45, 0x09, 0x08, 0x08, 
        0x26, 0x2A, 0xF3, 6, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F, 
        0xED, 2, 0x1B, 0x0B, 0xAE, 1, 0x77, 0xCD, 1, 0x63, 0x70, 
        9, 0x07, 0x07, 0x04, 0x0E, 0x0F, 0x09, 0x07, 0x08, 0x03, 
        0xE8, 1, 0x34, 0x62, 12, 0x18, 0x0D, 0x71, 0xED, 0x70, 0x70, 
        0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70, 0x63, 12, 0x18, 0x11, 
        0x71, 0xF1, 0x70, 0x70, 0x18, 0x13, 0x71, 0xF3, 0x70, 0x70, 
        0x64, 7, 0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07, 0x66, 
        10, 0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00, 
        0x67, 10, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 
        0x32, 0x98, 0x74, 7, 0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00, 
        0x98, 2, 0x3e, 0x07, 0x35, 0, 0x21, 0, 0x11, 0x80|120, 
        0x29, 0x80|20, 0x36, 1, 0x68 
    };
    const uint8_t *p = init_cmds;
    while(p < init_cmds + sizeof(init_cmds)) {
        lcd_write_cmd(*p++);
        uint8_t count = *p++;
        if(count & 0x80) { 
            sleep_ms(count & 0x7F); 
        } else { 
            while(count--) lcd_write_data(*p++); 
        }
    }
    lcd_clear_screen(0x0000); 
}

uint8_t get_lipo_percentage(float voltage) {
    if (voltage >= LIPO_LUT[0]) return 100;
    if (voltage <= LIPO_LUT[10]) return 0;
    for (int i = 0; i < 10; i++) {
        if (voltage <= LIPO_LUT[i] && voltage > LIPO_LUT[i+1]) {
            float v_range = LIPO_LUT[i] - LIPO_LUT[i+1];
            float v_delta = voltage - LIPO_LUT[i+1];
            return (100 - (i + 1) * 10) + (uint8_t)((v_delta / v_range) * 10.0f);
        }
    }
    return 0; 
}

void read_battery() {
    // Enable ADC only when needed
    hw_set_bits(&adc_hw->cs, ADC_CS_EN_BITS);
    adc_select_input(3);
    sleep_us(100); // Settling time
    
    uint32_t raw_sum = 0;
    for (int i = 0; i < 4; i++) { // Reduced from 10 to 4 samples for power savings
        raw_sum += adc_read(); 
        sleep_us(100); 
    }
    
    // Disable ADC to save power
    hw_clear_bits(&adc_hw->cs, ADC_CS_EN_BITS);
    
    float instant_voltage = (raw_sum / 4) * (3.3f / (1 << 12) * 3 * 1.06f);
    if (battery_avg == 0.0f) battery_avg = instant_voltage;
    else battery_avg = (battery_avg * 0.90f) + (instant_voltage * 0.10f);
    
    battery_voltage = battery_avg;
    
    if (battery_voltage > 4.75f) {
        is_charging = true;
    } else {
        is_charging = false;
    }
    
    battery_pct = get_lipo_percentage(battery_voltage);
    
    lv_color_t color_icon;
    if (is_charging) {
        color_icon = lv_color_hex(0x0000FF); // Blue
    } else if (battery_pct > 80) {
        color_icon = lv_color_hex(0x00FF00); // Green
    } else if (battery_pct < 20) {
        color_icon = lv_color_hex(0xFF0000); // Red
    } else {
        color_icon = lv_color_white();       // White
    }
    
    if (g_batt_cont) {
         for (int i = 0; i < 5; i++) {
            if (g_batt_bars[i]) {
                if (battery_pct > (i * 20)) { 
                    lv_obj_set_style_bg_color(g_batt_bars[i], color_icon, 0); 
                    lv_obj_set_style_bg_opa(g_batt_bars[i], LV_OPA_COVER, 0); 
                } else { 
                    lv_obj_set_style_bg_color(g_batt_bars[i], lv_color_hex(0x444444), 0);
                    lv_obj_set_style_bg_opa(g_batt_bars[i], LV_OPA_COVER, 0); 
                }
            }
        }
        lv_obj_set_style_border_color(g_batt_cont, color_icon, 0);
    }
}

bool touch_read(uint8_t *x, uint8_t *y) {
    if (!touch_found) return false;
    uint8_t reg = 0x00; uint8_t data[7];
    if (i2c_write_blocking(I2C_PORT, TOUCH_ADDR, &reg, 1, true) < 0) return false; 
    if (i2c_read_blocking(I2C_PORT, TOUCH_ADDR, data, 7, false) < 0) return false;
    
    uint8_t fingers = data[2] & 0x0F;
    if (fingers > 0) { 
        *x = data[4]; 
        *y = data[6]; 
        return true; 
    }
    return false;
}

void read_imu(void) {
    if (!imu_found) return;
    uint8_t reg = 0x35; uint8_t buf[6];
    i2c_write_blocking(I2C_PORT, detected_imu_addr, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, detected_imu_addr, buf, 6, false);
    acc_x = (int16_t)((buf[1] << 8) | buf[0]);
    acc_y = (int16_t)((buf[3] << 8) | buf[2]);
    acc_z = -(int16_t)((buf[5] << 8) | buf[4]);
}

void imu_set_low_power_mode(bool enable) {
    if (!imu_found) return;
    
    if (enable) {
        // Set accelerometer to low power mode (~50Hz instead of 1kHz)
        uint8_t cmd_low_pwr[2] = {0x02, 0x23}; // Lower ODR for power saving
        int result = i2c_write_blocking(I2C_PORT, detected_imu_addr, cmd_low_pwr, 2, false);
        if (result < 0) {
            // I2C error, but don't halt - IMU might be temporarily busy
            return;
        }
    } else {
        // Restore normal power mode
        uint8_t cmd_normal[2] = {0x02, 0x60}; // Normal ODR (1kHz)
        int result = i2c_write_blocking(I2C_PORT, detected_imu_addr, cmd_normal, 2, false);
        if (result < 0) {
            // I2C error, but don't halt
            return;
        }
    }
}

void check_usb_sync(void) {
    static char rx_buf[8];
    static int rx_idx = 0;

    while (true) {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) break;
        
        if (c == '?') { 
            printf("RP Watch\n"); 
            stdio_flush(); 
            rx_idx = 0; 
        } 
        else if (c == 'B') { 
            printf("Rebooting to BOOTSEL...\n");
            stdio_flush(); 
            sleep_ms(100); 
            reset_usb_boot(0, 0); 
        } 
        else if (c == 'L') {
            printf("LOG_START\n");
            printf("Today: %lu\n", settings.steps_today + session_steps);
            printf("Total: %lu\n", settings.total_lifetime_steps + session_steps);
            for(int i=0; i<7; i++) { 
                printf("Day-%d: %lu\n", i+1, settings.step_history[i]); 
            }
            printf("LOG_END\n");
            stdio_flush();
            rx_idx = 0;
        } 
        else if (c == 'T') {
            char buf[16];
            int idx = 0;
            while(idx < 15) {
                int ch = getchar_timeout_us(100000); 
                if (ch == PICO_ERROR_TIMEOUT) break;
                if (ch >= '0' && ch <= '9') {
                    buf[idx++] = ch;
                }
            }
            if (idx == 15) {
                 int year = (buf[0]-'0')*1000 + (buf[1]-'0')*100 + (buf[2]-'0')*10 + (buf[3]-'0');
                 int month = (buf[4]-'0')*10 + (buf[5]-'0');
                 int day = (buf[6]-'0')*10 + (buf[7]-'0');
                 int hour = (buf[8]-'0')*10 + (buf[9]-'0');
                 int min = (buf[10]-'0')*10 + (buf[11]-'0');
                 int sec = (buf[12]-'0')*10 + (buf[13]-'0');
                 datetime_t t_new = { .year = (int16_t)year, .month = (int8_t)month, .day = (int8_t)day, .dotw = 0, .hour = (int8_t)hour, .min = (int8_t)min, .sec = (int8_t)sec };
                 rtc_set_datetime(&t_new);
                 is_date_synced = true; // Mark date as valid
                 printf("ACK_SYNC\n");
                 stdio_flush();
            }
            rx_idx = 0;
        }
        else {
            if (c >= 32 && c <= 126 && rx_idx < 7) { 
                rx_buf[rx_idx++] = (char)c;
                rx_buf[rx_idx] = '\0';
                if (strstr(rx_buf, "WHO")) {
                    printf("RP Watch\n");
                    stdio_flush();
                    rx_idx = 0; 
                }
            } else {
                rx_idx = 0;
            }
        }
    }
}

void update_step_counter(void) {
    // High-pass filtered magnitude with hysteresis to suppress DC gravity and noise
    static uint32_t prev_mag = 0;
    static float hp_mag = 0.0f;
    static bool step_armed = false;

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    uint32_t mag = (uint32_t)sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);

    // High-pass filter: y[n] = a * (y[n-1] + x[n] - x[n-1])
    float delta = (float)((int32_t)mag - (int32_t)prev_mag);
    hp_mag = STEP_HP_ALPHA * (hp_mag + delta);
    prev_mag = mag;

    float abs_hp = fabsf(hp_mag);

    // Thresholds with user sensitivity and floor
    float high_thresh = STEP_HP_BASE - (float)settings.imu_sens * 30.0f;
    if (high_thresh < STEP_HP_MIN) high_thresh = STEP_HP_MIN;
    float low_thresh = high_thresh * STEP_HYST_LOW_FACTOR;

    if (!step_armed) {
        // Arm on strong upward activity
        if (abs_hp > high_thresh) {
            step_armed = true;
        }
    } else {
        // Count when activity settles back below low threshold
        if (abs_hp < low_thresh) {
            if (now_ms - last_step_time > STEP_DEBOUNCE_MS) {
                session_steps++;
                last_step_time = now_ms;

            // Update pilot face step counter
            uint32_t current_steps = settings.steps_today + session_steps;
            if (g_step_arc) {
                lv_arc_set_value(g_step_arc, current_steps > settings.step_goal ? settings.step_goal : current_steps);
            }
            if (g_step_label) {
                char buf[16]; snprintf(buf, sizeof(buf), "%lu", current_steps);
                lv_label_set_text(g_step_label, buf);
            }
            
            // Update activity screen arc and labels
            if (g_activity_arc) {
                lv_arc_set_value(g_activity_arc, current_steps > settings.step_goal ? settings.step_goal : current_steps);
            }
            if (g_activity_steps_label) {
                char buf[16]; snprintf(buf, sizeof(buf), "%lu", current_steps);
                lv_label_set_text(g_activity_steps_label, buf);
            }
            if (g_activity_pct_label) {
                char buf[8];
                uint8_t pct = (current_steps * 100) / settings.step_goal;
                if (pct > 100) pct = 100;
                snprintf(buf, sizeof(buf), "%u%%", pct);
                lv_label_set_text(g_activity_pct_label, buf);
            }
            }
            step_armed = false;
        }
    }
}

void check_lift_to_wake(void) {
    int16_t dynamic_threshold = 4000 - (settings.lift_sens * 30);
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    // Arm the trigger when arm is lowered
    if (acc_z < LIFT_ARM_THRES) {
        lift_trigger_armed = true;
        lift_arm_time = now;
    }
    
    // Reset armed state if timeout (3 seconds without reaching threshold)
    if (lift_trigger_armed && (now - lift_arm_time > 3000)) {
        lift_trigger_armed = false;
    }
    
    // Wake from sleep if armed and threshold reached
    if (pwr_state == PWR_SLEEP && lift_trigger_armed) {
        if (acc_z > dynamic_threshold) {
            set_power_state(PWR_ACTIVE);
            last_activity_time = now;
            lift_trigger_armed = false;
        }
    }
    
    // Also reset armed state when returning to active (manual wake via touch)
    if (pwr_state == PWR_ACTIVE && lift_trigger_armed) {
        lift_trigger_armed = false;
    }
}

void update_clock_ui(uint32_t now_ms) {
    if (t_now.sec != prev_sec) {
        prev_sec = t_now.sec;
        last_sec_time = now_ms;
    }
    
    if (g_line_sec) {
        float s_angle_deg = (t_now.sec * 6);
        float s_angle = (s_angle_deg - 90) * (PI / 180.0f);
        static lv_point_t p_sec[2];
        p_sec[0].x = 120; p_sec[0].y = 120;
        p_sec[1].x = 120 + (int)(cos(s_angle) * 110);
        p_sec[1].y = 120 + (int)(sin(s_angle) * 110);
        lv_line_set_points(g_line_sec, p_sec, 2);

        float m_angle = (t_now.min * 6 - 90) * (PI / 180.0f);
        static lv_point_t p_min[2];
        p_min[0].x = 120; p_min[0].y = 120;
        p_min[1].x = 120 + (int)(cos(m_angle) * 90);
        p_min[1].y = 120 + (int)(sin(m_angle) * 90);
        lv_line_set_points(g_line_min, p_min, 2);

        float h_val = (t_now.hour % 12) + (t_now.min / 60.0f);
        float h_angle = (h_val * 30 - 90) * (PI / 180.0f);
        static lv_point_t p_hour[2];
        p_hour[0].x = 120; p_hour[0].y = 120;
        p_hour[1].x = 120 + (int)(cos(h_angle) * 60);
        p_hour[1].y = 120 + (int)(sin(h_angle) * 60);
        lv_line_set_points(g_line_hour, p_hour, 2);
    }
    
    if (g_day_label) {
        if(is_date_synced) {
            int dotw = t_now.dotw; 
            if (dotw >= 0 && dotw <= 6) lv_label_set_text(g_day_label, DAY_NAMES[dotw]);
        } else {
            lv_label_set_text(g_day_label, "---");
        }
    }
    if (g_month_label && g_date_num) {
         if(is_date_synced) {
             int mon = t_now.month; int day = t_now.day;
             if (mon >= 1 && mon <= 12) {
                 lv_label_set_text(g_month_label, MONTH_NAMES[mon]);
                 lv_label_set_text_fmt(g_date_num, "%d", day);
             }
         } else {
             lv_label_set_text(g_month_label, "---");
             lv_label_set_text(g_date_num, "--");
         }
    }
    
    // Update digital 12H clock
    if (g_time_label) {
        int hour_12 = t_now.hour % 12;
        if (hour_12 == 0) hour_12 = 12;
        const char *ampm = (t_now.hour < 12) ? "AM" : "PM";
        lv_label_set_text_fmt(g_time_label, "%d:%02d %s", hour_12, t_now.min, ampm);
    }
}

// EVENTS
static void event_brightness_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * slider = lv_event_get_target(e);
    
    if (code == LV_EVENT_VALUE_CHANGED) {
        int val = lv_slider_get_value(slider);
        settings.brightness = (uint8_t)val;
        set_backlight(settings.brightness);
        // Mark settings as changed, will save after debounce
        settings_needs_save = true;
        settings_changed_time = to_ms_since_boot(get_absolute_time());
    }
}
static void event_imu_sens_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * slider = lv_event_get_target(e);
    if (code == LV_EVENT_VALUE_CHANGED) {
        settings.imu_sens = (uint8_t)lv_slider_get_value(slider);
        // Mark settings as changed, will save after debounce
        settings_needs_save = true;
        settings_changed_time = to_ms_since_boot(get_absolute_time());
    }
}
static void event_lift_sens_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * slider = lv_event_get_target(e);
    if (code == LV_EVENT_VALUE_CHANGED) {
        settings.lift_sens = (uint8_t)lv_slider_get_value(slider);
        // Mark settings as changed, will save after debounce
        settings_needs_save = true;
        settings_changed_time = to_ms_since_boot(get_absolute_time());
    }
}

// --- STOPWATCH LOGIC ---
void update_stopwatch_logic(void) {
    if(sw_running) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        uint32_t elapsed = sw_accumulated_time + (now - sw_start_time);
        
        uint32_t total_sec = elapsed / 1000;
        uint32_t hsec = (elapsed % 1000) / 10;
        uint32_t min = total_sec / 60;
        uint32_t sec = total_sec % 60;
        
        if(g_sw_label_time) {
            lv_label_set_text_fmt(g_sw_label_time, "%02lu:%02lu.%02lu", min, sec, hsec);
        }
    }
}

static void event_sw_toggle(lv_event_t * e) {
    if(sw_running) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        sw_accumulated_time += (now - sw_start_time);
        sw_running = false;
        if(g_sw_label_toggle) lv_label_set_text(g_sw_label_toggle, "Start");
        if(g_sw_btn_toggle) lv_obj_set_style_bg_color(g_sw_btn_toggle, lv_color_hex(0x008000), 0);
    } else {
        sw_start_time = to_ms_since_boot(get_absolute_time());
        sw_running = true;
        if(g_sw_label_toggle) lv_label_set_text(g_sw_label_toggle, "Stop");
        if(g_sw_btn_toggle) lv_obj_set_style_bg_color(g_sw_btn_toggle, lv_color_hex(0xFF0000), 0);
    }
}

static void event_sw_reset(lv_event_t * e) {
    if(!sw_running) {
        sw_accumulated_time = 0;
        if(g_sw_label_time) lv_label_set_text(g_sw_label_time, "00:00.00");
        sw_lap_buffer[0] = '\0';
        if(g_sw_label_laps) lv_label_set_text(g_sw_label_laps, "");
    } else {
        if(g_sw_label_time && g_sw_label_laps) {
            const char* current_time = lv_label_get_text(g_sw_label_time);
            if(strlen(sw_lap_buffer) + strlen(current_time) + 2 < sizeof(sw_lap_buffer)) {
                strcat(sw_lap_buffer, current_time);
                strcat(sw_lap_buffer, "\n");
                lv_label_set_text(g_sw_label_laps, sw_lap_buffer);
            }
        }
    }
}

static void event_alarm_save_handler(lv_event_t * e) {
    if(g_alarm_roller_h && g_alarm_roller_m && g_alarm_roller_ap && g_alarm_roller_mode) {
        int h_12 = lv_roller_get_selected(g_alarm_roller_h) + 1; 
        int m = lv_roller_get_selected(g_alarm_roller_m);
        int ampm = lv_roller_get_selected(g_alarm_roller_ap); 
        int mode = lv_roller_get_selected(g_alarm_roller_mode); 
        
        if (ampm == 1 && h_12 != 12) h_12 += 12; 
        if (ampm == 0 && h_12 == 12) h_12 = 0;   
        
        settings.alarm_h = h_12;
        settings.alarm_m = m;
        settings.alarm_mode = mode;
        save_settings();
    }
}

static void event_alarm_toggle_handler(lv_event_t * e) {
    lv_obj_t * sw = lv_event_get_target(e);
    if(lv_obj_has_state(sw, LV_STATE_CHECKED)) settings.alarm_enabled = true;
    else settings.alarm_enabled = false;
    save_settings();
}

static void event_alarm_dismiss(lv_event_t * e) {
    alarm_firing = false;
    if(g_alarm_modal) {
        lv_msgbox_close(g_alarm_modal);
        g_alarm_modal = NULL;
    }
    if(settings.alarm_mode == ALARM_MODE_ONCE) {
        settings.alarm_enabled = false;
        save_settings(); 
        if(g_alarm_sw) lv_obj_clear_state(g_alarm_sw, LV_STATE_CHECKED);
    }
}

void check_alarm_trigger(void) {
    if (settings.alarm_enabled && !alarm_firing && t_now.sec == 0) {
        if (t_now.hour == settings.alarm_h && t_now.min == settings.alarm_m) {
            bool should_fire = false;
            if (settings.alarm_mode == ALARM_MODE_ONCE) should_fire = true;
            else if (settings.alarm_mode == ALARM_MODE_DAILY) should_fire = true;
            else if (settings.alarm_mode == ALARM_MODE_WEEKDAY) { if (t_now.dotw >= 1 && t_now.dotw <= 5) should_fire = true; }
            else if (settings.alarm_mode == ALARM_MODE_WEEKEND) { if (t_now.dotw == 0 || t_now.dotw == 6) should_fire = true; }
            
            if (should_fire) {
                alarm_firing = true;
                set_power_state(PWR_ACTIVE); 
                static const char * btns[] = {"Dismiss", ""};
                g_alarm_modal = lv_msgbox_create(NULL, "ALARM", "Wake Up!", btns, true);
                lv_obj_center(g_alarm_modal);
                lv_obj_set_style_bg_color(g_alarm_modal, lv_color_hex(0xAA0000), 0);
                lv_obj_add_event_cb(g_alarm_modal, event_alarm_dismiss, LV_EVENT_VALUE_CHANGED, NULL);
            }
        }
    }
    if (alarm_firing) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - alarm_last_beep > 200) {
            alarm_beep_state = !alarm_beep_state;
            if (alarm_beep_state) play_tone(6300, 200);
            alarm_last_beep = now;
        }
    }
}

static void event_timer_dismiss(lv_event_t * e) {
    timer_state = TMR_SETUP; 
    if(g_tmr_modal) {
        lv_msgbox_close(g_tmr_modal);
        g_tmr_modal = NULL;
    }
    if(g_tmr_cont_setup) lv_obj_clear_flag(g_tmr_cont_setup, LV_OBJ_FLAG_HIDDEN);
    if(g_tmr_cont_run) lv_obj_add_flag(g_tmr_cont_run, LV_OBJ_FLAG_HIDDEN);
}

void update_timer_logic(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (timer_state == TMR_RUNNING) {
        if (now - timer_last_tick >= 1000) {
            if (timer_remaining_sec > 0) {
                timer_remaining_sec--;
                if (g_tmr_lbl_digits) {
                    uint32_t h = timer_remaining_sec / 3600;
                    uint32_t m = (timer_remaining_sec % 3600) / 60;
                    uint32_t s = timer_remaining_sec % 60;
                    if (h > 0) lv_label_set_text_fmt(g_tmr_lbl_digits, "%d:%02d:%02d", h, m, s);
                    else lv_label_set_text_fmt(g_tmr_lbl_digits, "%02d:%02d", m, s);
                }
                if (g_tmr_arc && timer_duration_sec > 0) {
                    int pct = (timer_remaining_sec * 100) / timer_duration_sec;
                    lv_arc_set_value(g_tmr_arc, pct);
                }
            } else {
                timer_state = TMR_FIRING;
                set_power_state(PWR_ACTIVE);
                static const char * btns[] = {"OK", ""};
                g_tmr_modal = lv_msgbox_create(NULL, "TIMER", "Time's Up!", btns, true);
                lv_obj_center(g_tmr_modal);
                lv_obj_set_style_bg_color(g_tmr_modal, lv_color_hex(0xFF8800), 0);
                lv_obj_add_event_cb(g_tmr_modal, event_timer_dismiss, LV_EVENT_VALUE_CHANGED, NULL);
            }
            timer_last_tick = now;
        }
    }
    if (timer_state == TMR_FIRING) {
        if (now - alarm_last_beep > 100) {
            alarm_beep_state = !alarm_beep_state;
            if (alarm_beep_state) play_tone(6300, 100); 
            alarm_last_beep = now;
        }
    }
}

static void event_timer_toggle_handler(lv_event_t * e) {
    if (timer_state == TMR_SETUP) {
        if (g_tmr_roller_h && g_tmr_roller_m && g_tmr_roller_s) {
            int h = lv_roller_get_selected(g_tmr_roller_h);
            int m = lv_roller_get_selected(g_tmr_roller_m);
            int s = lv_roller_get_selected(g_tmr_roller_s);
            uint32_t total_sec = (h * 3600) + (m * 60) + s;
            if (total_sec > 0) {
                settings.timer_h = h; settings.timer_m = m; settings.timer_s = s;
                save_settings();
                timer_duration_sec = total_sec;
                timer_remaining_sec = total_sec;
                timer_state = TMR_RUNNING;
                timer_last_tick = to_ms_since_boot(get_absolute_time());
                lv_obj_add_flag(g_tmr_cont_setup, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(g_tmr_cont_run, LV_OBJ_FLAG_HIDDEN);
                if(g_tmr_btn_pause) { lv_obj_t * l = lv_obj_get_child(g_tmr_btn_pause, 0); lv_label_set_text(l, "Pause"); }
                if (g_tmr_arc) lv_arc_set_value(g_tmr_arc, 100);
                if (g_tmr_lbl_digits) {
                    if (h > 0) lv_label_set_text_fmt(g_tmr_lbl_digits, "%d:%02d:%02d", h, m, s);
                    else lv_label_set_text_fmt(g_tmr_lbl_digits, "%02d:%02d", m, s);
                }
            }
        }
    }
}

static void event_timer_pause_handler(lv_event_t * e) {
    if (timer_state == TMR_RUNNING) {
        timer_state = TMR_PAUSED;
        lv_obj_t * l = lv_obj_get_child(e->current_target, 0);
        lv_label_set_text(l, "Resume");
    } else if (timer_state == TMR_PAUSED) {
        timer_state = TMR_RUNNING;
        timer_last_tick = to_ms_since_boot(get_absolute_time()); 
        lv_obj_t * l = lv_obj_get_child(e->current_target, 0);
        lv_label_set_text(l, "Pause");
    }
}

static void event_timer_reset_handler(lv_event_t * e) {
    timer_state = TMR_SETUP;
    lv_obj_add_flag(g_tmr_cont_run, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(g_tmr_cont_setup, LV_OBJ_FLAG_HIDDEN);
}

static void event_face_btn_handler(lv_event_t * e) {
    lv_obj_t * btn = lv_event_get_target(e);
    if (timer_state == TMR_FIRING) {
        timer_state = TMR_SETUP;
        alarm_beep_state = false;
        if(g_tmr_modal) { lv_msgbox_close(g_tmr_modal); g_tmr_modal = NULL; }
    }
}

// --- CLOCK SET LOGIC ---
static void event_time_save_handler(lv_event_t * e) {
    if(g_roller_hour && g_roller_min) {
        int h = lv_roller_get_selected(g_roller_hour);
        int m = lv_roller_get_selected(g_roller_min);
        
        // Validate hour and minute ranges
        if (h < 0 || h > 23) h = 12; // Default to noon if invalid
        if (m < 0 || m > 59) m = 0;  // Default to :00 if invalid
        
        // Preserve current date (even if invalid) so we don't reset to 1970
        datetime_t now;
        rtc_get_datetime(&now);
        
        datetime_t new_t = {
            .year = now.year,
            .month = now.month,
            .day = now.day,
            .dotw = now.dotw,
            .hour = (int8_t)h,
            .min = (int8_t)m,
            .sec = 0
        };
        
        rtc_set_datetime(&new_t);
        is_date_synced = false; // Mark date as invalid since manual set
        
        // Visual Feedback (Flash green)
        lv_obj_t * btn = lv_event_get_target(e);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x00FF00), 0);
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        lv_label_set_text(label, "Saved!");
    }
}

// --- DIAGNOSTICS LOGIC ---
void update_diagnostics_logic(void) {
    if (g_diag_label) {
        struct mallinfo m = mallinfo();
        char buf[256];
        snprintf(buf, sizeof(buf), 
            "ADC: %.2fv\n"
            "SoC: %d%%\n"
            "USB: %s\n"
            "Heap Free: %d B\n"
            "CPU Load: %lu%%\n\n"
            "FW: %s",
            battery_voltage,
            battery_pct,
            is_charging ? "Connected" : "Battery",
            m.fordblks,
            cpu_load_pct,
            FIRMWARE_VERSION
        );
        lv_label_set_text(g_diag_label, buf);
    }
}

// --- DRIVER RESTORATION ---
static void lvgl_display_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = lv_area_get_width(area);
    uint32_t h = lv_area_get_height(area);
    lcd_set_window(area->x1, area->y1, w, h);
    gpio_put(LCD_DC_PIN, 1);
    gpio_put(LCD_CS_PIN, 0);
    spi_write_blocking(LCD_SPI_PORT, (uint8_t *)color_p, w * h * 2);
    gpio_put(LCD_CS_PIN, 1);
    lv_disp_flush_ready(disp_drv);
}

static void lvgl_indev_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
    uint8_t x, y;
    if (touch_read(&x, &y)) {
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = y;
        data->point.y = 240 - x;
        last_activity_time = to_ms_since_boot(get_absolute_time());
        if (pwr_state != PWR_ACTIVE) set_power_state(PWR_ACTIVE);
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static bool lv_tick_timer_cb(struct repeating_timer *t) { 
    lv_tick_inc(1); // Keep the tick going
    return true; 
}

void init_lvgl_tick(void) {
    static struct repeating_timer timer;
    add_repeating_timer_ms(1, lv_tick_timer_cb, NULL, &timer);
}

void lvgl_init_drivers(void) {
    lv_disp_draw_buf_init(&disp_buf, buf_1, NULL, DRAW_BUFFER_SIZE); // Single buffer
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_HOR_RES;
    disp_drv.ver_res = LCD_VER_RES;
    disp_drv.flush_cb = lvgl_display_flush;
    disp_drv.draw_buf = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lvgl_indev_read;
    lv_indev_drv_register(&indev_drv);
}

// Global Cleaners
void clean_pilot_globals(void) {
    g_step_arc = NULL; 
    g_step_label = NULL; g_time_label = NULL; g_line_hour = NULL; g_line_min = NULL; 
    g_line_sec = NULL; g_day_label = NULL; g_month_label = NULL; 
    g_date_box = NULL; g_date_num = NULL; 
    for(int i=0; i<5; i++) g_batt_bars[i] = NULL; 
    g_batt_cont = NULL;
}
void clean_tools_globals(void) {
    g_sw_label_time = NULL; g_sw_btn_toggle = NULL; 
    g_sw_label_toggle = NULL; g_sw_label_laps = NULL; 
    g_diag_label = NULL; 
}
void clean_activity_globals(void) {
    g_activity_arc = NULL;
    g_activity_steps_label = NULL;
    g_activity_goal_label = NULL;
    g_activity_pct_label = NULL;
}
void clean_timeset_globals(void) {
    g_roller_hour = NULL;
    g_roller_min = NULL;
}

// --- UI BUILDER FUNCTIONS ---

void build_face_pilot(lv_obj_t *parent) {
    static lv_point_t tick_points[12][2]; 
    static lv_style_t style_tick;
    static bool style_inited = false;
    
    if(!style_inited) {
        lv_style_init(&style_tick);
        lv_style_set_line_width(&style_tick, 3);
        lv_style_set_line_color(&style_tick, lv_color_white());
        style_inited = true;
    }

    for (int i = 0; i < 12; i++) {
        float angle = (i * 30 - 90) * (PI / 180.0f);
        int r_outer = 119; 
        int r_inner = 105; 
        
        tick_points[i][0].x = 120 + (int)(cos(angle) * r_outer);
        tick_points[i][0].y = 120 + (int)(sin(angle) * r_outer);
        tick_points[i][1].x = 120 + (int)(cos(angle) * r_inner);
        tick_points[i][1].y = 120 + (int)(sin(angle) * r_inner);
        
        lv_obj_t *line = lv_line_create(parent);
        lv_line_set_points(line, tick_points[i], 2);
        lv_obj_add_style(line, &style_tick, 0);
    }
    
    int r_num = 94; 
    for (int i = 1; i <= 12; i++) {
        float angle = (i * 30 - 90) * (PI / 180.0f);
        int x = 120 + (int)(cos(angle) * r_num);
        int y = 120 + (int)(sin(angle) * r_num);
        lv_obj_t *label = lv_label_create(parent);
        lv_label_set_text_fmt(label, "%d", i);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_obj_set_pos(label, x - 5, y - 8); 
    }

    g_day_label = lv_label_create(parent); 
    lv_label_set_text(g_day_label, "---"); 
    lv_obj_align(g_day_label, LV_ALIGN_LEFT_MID, 40, 0); 
    lv_obj_set_style_text_color(g_day_label, lv_color_hex(0x00FF00), 0); 

    g_month_label = lv_label_create(parent); 
    lv_label_set_text(g_month_label, "---"); 
    lv_obj_set_style_text_color(g_month_label, lv_color_hex(0x00FF00), 0); 
    lv_obj_align(g_month_label, LV_ALIGN_RIGHT_MID, -65, 0); 
    
    g_date_box = lv_obj_create(parent); 
    lv_obj_set_size(g_date_box, 30, 24); 
    lv_obj_align(g_date_box, LV_ALIGN_RIGHT_MID, -30, 0);
    lv_obj_set_style_bg_color(g_date_box, lv_color_black(), 0); 
    lv_obj_set_style_border_color(g_date_box, lv_color_hex(0x00FF00), 0); 
    lv_obj_set_style_border_width(g_date_box, 2, 0); 
    lv_obj_set_style_radius(g_date_box, 2, 0);

    g_date_num = lv_label_create(g_date_box); 
    lv_label_set_text(g_date_num, "--"); 
    lv_obj_center(g_date_num); 
    lv_obj_set_style_text_color(g_date_num, lv_color_white(), 0);
    
    // Digital 12H Clock Display
    g_time_label = lv_label_create(parent);
    lv_obj_set_style_text_font(g_time_label, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(g_time_label, lv_color_white(), 0);
    // Move up by ~half the font height (~12px for 24pt)
    lv_obj_align(g_time_label, LV_ALIGN_CENTER, 0, -32);
    
    // Hands
    static lv_style_t style_line_hour; 
    lv_style_init(&style_line_hour); 
    lv_style_set_line_width(&style_line_hour, 6); 
    lv_style_set_line_color(&style_line_hour, lv_color_hex(0xFF0000)); 
    lv_style_set_line_rounded(&style_line_hour, true); 
    g_line_hour = lv_line_create(parent); 
    lv_obj_add_style(g_line_hour, &style_line_hour, 0);

    static lv_style_t style_line_min; 
    lv_style_init(&style_line_min); 
    lv_style_set_line_width(&style_line_min, 4); 
    lv_style_set_line_color(&style_line_min, lv_color_hex(0x4444FF)); 
    lv_style_set_line_rounded(&style_line_min, true); 
    g_line_min = lv_line_create(parent); 
    lv_obj_add_style(g_line_min, &style_line_min, 0);

    static lv_style_t style_line_sec; 
    lv_style_init(&style_line_sec); 
    lv_style_set_line_width(&style_line_sec, 2); 
    lv_style_set_line_color(&style_line_sec, lv_color_hex(0xFF0000)); 
    g_line_sec = lv_line_create(parent); 
    lv_obj_add_style(g_line_sec, &style_line_sec, 0);

    // Battery Icon
    g_batt_cont = lv_obj_create(parent); 
    lv_obj_set_size(g_batt_cont, 34, 18); 
    lv_obj_align(g_batt_cont, LV_ALIGN_CENTER, 0, 50); 
    lv_obj_set_style_bg_color(g_batt_cont, lv_color_black(), 0); 
    lv_obj_set_style_border_color(g_batt_cont, lv_color_white(), 0); 
    lv_obj_set_style_border_width(g_batt_cont, 2, 0);
    lv_obj_set_style_pad_all(g_batt_cont, 2, 0); 
    lv_obj_set_style_radius(g_batt_cont, 2, 0);
    
    lv_obj_t *nub = lv_obj_create(parent); 
    lv_obj_set_size(nub, 4, 8); 
    lv_obj_align_to(nub, g_batt_cont, LV_ALIGN_OUT_RIGHT_MID, 0, 0);
    lv_obj_set_style_bg_color(nub, lv_color_white(), 0); 
    lv_obj_set_style_border_width(nub, 0, 0);

    for(int i=0; i<5; i++) {
        g_batt_bars[i] = lv_obj_create(g_batt_cont); 
        lv_obj_set_size(g_batt_bars[i], 4, 10); 
        lv_obj_set_pos(g_batt_bars[i], i * 6, 0); 
        lv_obj_set_style_border_width(g_batt_bars[i], 0, 0); 
        lv_obj_set_style_radius(g_batt_bars[i], 0, 0);
        lv_obj_set_style_bg_color(g_batt_bars[i], lv_color_hex(0x444444), 0); 
    }
    
    // Digital Clock Display (removed arc - keeping it on activity screen)
    // Clock hands are rendered via update_clock_ui()
}

void build_tools_page(lv_obj_t *parent) {
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, 240, 240);
    lv_obj_center(cont);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(cont, LV_DIR_VER);
    lv_obj_set_scroll_snap_y(cont, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(cont, lv_color_black(), 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLL_ELASTIC); 
    lv_obj_add_flag(cont, LV_OBJ_FLAG_SCROLL_ONE);       
    lv_obj_set_style_pad_all(cont, 0, 0);                
    lv_obj_set_style_pad_gap(cont, 0, 0);                

    // Page 1: Stopwatch
    lv_obj_t *page1 = lv_obj_create(cont);
    lv_obj_set_size(page1, 240, 240);
    lv_obj_set_style_bg_color(page1, lv_color_black(), 0);
    lv_obj_set_style_border_width(page1, 0, 0);
    lv_obj_add_flag(page1, LV_OBJ_FLAG_SNAPPABLE); 
    
    g_sw_label_time = lv_label_create(page1);
    lv_label_set_text(g_sw_label_time, "00:00.00");
    lv_obj_set_style_text_font(g_sw_label_time, &lv_font_montserrat_14, 0);
    lv_obj_align(g_sw_label_time, LV_ALIGN_TOP_MID, 0, 50);
    lv_obj_set_style_text_color(g_sw_label_time, lv_color_white(), 0);

    g_sw_btn_toggle = lv_btn_create(page1);
    lv_obj_set_size(g_sw_btn_toggle, 80, 50);
    lv_obj_align(g_sw_btn_toggle, LV_ALIGN_CENTER, 50, 20); 
    lv_obj_set_style_bg_color(g_sw_btn_toggle, lv_color_hex(0x008000), 0);
    lv_obj_add_event_cb(g_sw_btn_toggle, event_sw_toggle, LV_EVENT_CLICKED, NULL);
    g_sw_label_toggle = lv_label_create(g_sw_btn_toggle);
    lv_label_set_text(g_sw_label_toggle, "Start");
    lv_obj_center(g_sw_label_toggle);

    lv_obj_t *btn_reset = lv_btn_create(page1);
    lv_obj_set_size(btn_reset, 80, 50);
    lv_obj_align(btn_reset, LV_ALIGN_CENTER, -50, 20); 
    lv_obj_set_style_bg_color(btn_reset, lv_color_hex(0x808080), 0);
    lv_obj_add_event_cb(btn_reset, event_sw_reset, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_reset = lv_label_create(btn_reset);
    lv_label_set_text(lbl_reset, "Rst");
    lv_obj_center(lbl_reset);
    
    lv_obj_t *lap_cont = lv_obj_create(page1);
    lv_obj_set_size(lap_cont, 160, 60);
    lv_obj_align(lap_cont, LV_ALIGN_BOTTOM_MID, 0, -10); 
    lv_obj_set_style_bg_opa(lap_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(lap_cont, 0, 0);
    lv_obj_clear_flag(lap_cont, LV_OBJ_FLAG_SCROLLABLE);

    g_sw_label_laps = lv_label_create(lap_cont);
    lv_label_set_text(g_sw_label_laps, "");
    lv_obj_align(g_sw_label_laps, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_text_color(g_sw_label_laps, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_style_text_align(g_sw_label_laps, LV_TEXT_ALIGN_CENTER, 0);

    // Other Pages
    lv_obj_t *page2 = lv_obj_create(cont);
    lv_obj_set_size(page2, 240, 240);
    lv_obj_set_style_bg_color(page2, lv_color_black(), 0);
    lv_obj_set_style_border_width(page2, 0, 0);
    lv_obj_add_flag(page2, LV_OBJ_FLAG_SNAPPABLE); 
    build_timer_page(page2);

    lv_obj_t *page3 = lv_obj_create(cont);
    lv_obj_set_size(page3, 240, 240);
    lv_obj_set_style_bg_color(page3, lv_color_black(), 0);
    lv_obj_set_style_border_width(page3, 0, 0);
    lv_obj_add_flag(page3, LV_OBJ_FLAG_SNAPPABLE); 
    build_alarm_page(page3);

    // PAGE 4: DIAGNOSTICS (Moved Version Here)
    lv_obj_t *page4 = lv_obj_create(cont);
    lv_obj_set_size(page4, 240, 240);
    lv_obj_set_style_bg_color(page4, lv_color_black(), 0);
    lv_obj_set_style_border_width(page4, 0, 0);
    lv_obj_add_flag(page4, LV_OBJ_FLAG_SNAPPABLE); 
    
    lv_obj_t *l4_title = lv_label_create(page4);
    lv_label_set_text(l4_title, "Diagnostics");
    lv_obj_align(l4_title, LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_set_style_text_color(l4_title, lv_color_hex(0x00FF00), 0); 

    g_diag_label = lv_label_create(page4);
    lv_label_set_text(g_diag_label, "Loading...");
    lv_obj_center(g_diag_label);
    lv_obj_set_style_text_color(g_diag_label, lv_color_white(), 0);
    lv_obj_set_style_text_align(g_diag_label, LV_TEXT_ALIGN_LEFT, 0);
}

void build_timer_page(lv_obj_t *parent) {
    g_tmr_cont_setup = lv_obj_create(parent);
    lv_obj_set_size(g_tmr_cont_setup, 240, 240);
    lv_obj_center(g_tmr_cont_setup);
    lv_obj_set_style_bg_color(g_tmr_cont_setup, lv_color_black(), 0);
    lv_obj_set_style_border_width(g_tmr_cont_setup, 0, 0);
    
    lv_obj_t * label = lv_label_create(g_tmr_cont_setup);
    lv_label_set_text(label, "Timer Setup");
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);

    lv_obj_t * row = lv_obj_create(g_tmr_cont_setup);
    lv_obj_set_size(row, 220, 120);
    lv_obj_align(row, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(row, 5, 0);

    g_tmr_roller_h = lv_roller_create(row);
    lv_roller_set_options(g_tmr_roller_h, "0\n1\n2\n3\n4\n5\n6\n7\n8\n9", LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(g_tmr_roller_h, 3);
    lv_obj_set_width(g_tmr_roller_h, 40);
    lv_obj_t *c1 = lv_label_create(row); lv_label_set_text(c1, ":"); lv_obj_set_style_text_color(c1, lv_color_white(), 0);

    g_tmr_roller_m = lv_roller_create(row);
    lv_roller_set_options(g_tmr_roller_m, 
        "00\n01\n02\n03\n04\n05\n06\n07\n08\n09\n10\n11\n12\n13\n14\n15\n16\n17\n18\n19\n"
        "20\n21\n22\n23\n24\n25\n26\n27\n28\n29\n30\n31\n32\n33\n34\n35\n36\n37\n38\n39\n"
        "40\n41\n42\n43\n44\n45\n46\n47\n48\n49\n50\n51\n52\n53\n54\n55\n56\n57\n58\n59", 
        LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(g_tmr_roller_m, 3);
    lv_obj_set_width(g_tmr_roller_m, 40);
    lv_obj_t *c2 = lv_label_create(row); lv_label_set_text(c2, ":"); lv_obj_set_style_text_color(c2, lv_color_white(), 0);

    g_tmr_roller_s = lv_roller_create(row);
    lv_roller_set_options(g_tmr_roller_s, "00\n05\n10\n15\n20\n25\n30\n35\n40\n45\n50\n55", LV_ROLLER_MODE_NORMAL); 
    lv_roller_set_visible_row_count(g_tmr_roller_s, 3);
    lv_obj_set_width(g_tmr_roller_s, 40);

    lv_roller_set_selected(g_tmr_roller_h, settings.timer_h, LV_ANIM_OFF);
    lv_roller_set_selected(g_tmr_roller_m, settings.timer_m, LV_ANIM_OFF);
    lv_roller_set_selected(g_tmr_roller_s, settings.timer_s / 5, LV_ANIM_OFF); 

    g_tmr_btn_start = lv_btn_create(g_tmr_cont_setup);
    lv_obj_set_size(g_tmr_btn_start, 80, 50);
    lv_obj_align(g_tmr_btn_start, LV_ALIGN_BOTTOM_MID, 0, -30);
    lv_obj_set_style_bg_color(g_tmr_btn_start, lv_color_hex(0x008000), 0);
    lv_obj_add_event_cb(g_tmr_btn_start, event_timer_toggle_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_t * l_start = lv_label_create(g_tmr_btn_start);
    lv_label_set_text(l_start, "START");
    lv_obj_center(l_start);

    g_tmr_cont_run = lv_obj_create(parent);
    lv_obj_set_size(g_tmr_cont_run, 240, 240);
    lv_obj_center(g_tmr_cont_run);
    lv_obj_set_style_bg_color(g_tmr_cont_run, lv_color_black(), 0);
    lv_obj_set_style_border_width(g_tmr_cont_run, 0, 0);
    lv_obj_add_flag(g_tmr_cont_run, LV_OBJ_FLAG_HIDDEN);

    g_tmr_arc = lv_arc_create(g_tmr_cont_run);
    lv_obj_set_size(g_tmr_arc, 220, 220);
    lv_obj_center(g_tmr_arc);
    lv_arc_set_rotation(g_tmr_arc, 270);
    lv_arc_set_bg_angles(g_tmr_arc, 0, 360);
    lv_arc_set_value(g_tmr_arc, 100);
    lv_obj_remove_style(g_tmr_arc, NULL, LV_PART_KNOB); 
    lv_obj_clear_flag(g_tmr_arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_color(g_tmr_arc, lv_color_hex(0x00FF00), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(g_tmr_arc, 10, LV_PART_INDICATOR);

    g_tmr_lbl_digits = lv_label_create(g_tmr_cont_run);
    lv_label_set_text(g_tmr_lbl_digits, "00:00");
    lv_obj_set_style_text_font(g_tmr_lbl_digits, &lv_font_montserrat_14, 0); 
    lv_obj_align(g_tmr_lbl_digits, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_style_text_color(g_tmr_lbl_digits, lv_color_white(), 0);

    g_tmr_btn_pause = lv_btn_create(g_tmr_cont_run);
    lv_obj_set_size(g_tmr_btn_pause, 80, 40);
    lv_obj_align(g_tmr_btn_pause, LV_ALIGN_BOTTOM_MID, -50, -40);
    lv_obj_set_style_bg_color(g_tmr_btn_pause, lv_color_hex(0xFF8800), 0);
    lv_obj_add_event_cb(g_tmr_btn_pause, event_timer_pause_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_t * l_pause = lv_label_create(g_tmr_btn_pause);
    lv_label_set_text(l_pause, "Pause");
    lv_obj_center(l_pause);

    lv_obj_t * btn_rst = lv_btn_create(g_tmr_cont_run);
    lv_obj_set_size(btn_rst, 80, 40);
    lv_obj_align(btn_rst, LV_ALIGN_BOTTOM_MID, 50, -40);
    lv_obj_set_style_bg_color(btn_rst, lv_color_hex(0x555555), 0);
    lv_obj_add_event_cb(btn_rst, event_timer_reset_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_t * l_rst = lv_label_create(btn_rst);
    lv_label_set_text(l_rst, "Stop");
    lv_obj_center(l_rst);
    
    if (timer_state == TMR_RUNNING || timer_state == TMR_PAUSED || timer_state == TMR_FIRING) {
        lv_obj_add_flag(g_tmr_cont_setup, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(g_tmr_cont_run, LV_OBJ_FLAG_HIDDEN);
        
        uint32_t h = timer_remaining_sec / 3600;
        uint32_t m = (timer_remaining_sec % 3600) / 60;
        uint32_t s = timer_remaining_sec % 60;
        if (h > 0) lv_label_set_text_fmt(g_tmr_lbl_digits, "%d:%02d:%02d", h, m, s);
        else lv_label_set_text_fmt(g_tmr_lbl_digits, "%02d:%02d", m, s);
        
        if (timer_duration_sec > 0) {
            int pct = (timer_remaining_sec * 100) / timer_duration_sec;
            lv_arc_set_value(g_tmr_arc, pct);
        }
        
        if (timer_state == TMR_PAUSED) lv_label_set_text(l_pause, "Resume");
        else lv_label_set_text(l_pause, "Pause");
    }
}

void build_alarm_page(lv_obj_t *parent) {
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, "Alarm");
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);

    lv_obj_t *cont_time = lv_obj_create(parent);
    lv_obj_set_size(cont_time, 200, 100);
    lv_obj_align(cont_time, LV_ALIGN_TOP_MID, 0, 30);
    lv_obj_set_style_bg_opa(cont_time, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cont_time, 0, 0);
    lv_obj_set_flex_flow(cont_time, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cont_time, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(cont_time, 0, 0);
    lv_obj_set_style_pad_gap(cont_time, 2, 0);

    g_alarm_roller_h = lv_roller_create(cont_time);
    lv_roller_set_options(g_alarm_roller_h, "1\n2\n3\n4\n5\n6\n7\n8\n9\n10\n11\n12", LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(g_alarm_roller_h, 3);
    lv_obj_set_width(g_alarm_roller_h, 40);
    lv_obj_add_event_cb(g_alarm_roller_h, event_alarm_save_handler, LV_EVENT_VALUE_CHANGED, NULL);

    g_alarm_roller_m = lv_roller_create(cont_time);
    lv_roller_set_options(g_alarm_roller_m, "00\n05\n10\n15\n20\n25\n30\n35\n40\n45\n50\n55", LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(g_alarm_roller_m, 3);
    lv_obj_set_width(g_alarm_roller_m, 40);
    lv_obj_add_event_cb(g_alarm_roller_m, event_alarm_save_handler, LV_EVENT_VALUE_CHANGED, NULL);

    g_alarm_roller_ap = lv_roller_create(cont_time);
    lv_roller_set_options(g_alarm_roller_ap, "AM\nPM", LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(g_alarm_roller_ap, 2);
    lv_obj_set_width(g_alarm_roller_ap, 40);
    lv_obj_add_event_cb(g_alarm_roller_ap, event_alarm_save_handler, LV_EVENT_VALUE_CHANGED, NULL);

    int h_24 = settings.alarm_h;
    int h_12 = h_24 % 12; 
    if (h_12 == 0) h_12 = 12;
    int is_pm = (h_24 >= 12) ? 1 : 0;
    
    lv_roller_set_selected(g_alarm_roller_h, h_12 - 1, LV_ANIM_OFF);
    lv_roller_set_selected(g_alarm_roller_m, settings.alarm_m / 5, LV_ANIM_OFF);
    lv_roller_set_selected(g_alarm_roller_ap, is_pm, LV_ANIM_OFF);

    lv_obj_t *cont_mode = lv_obj_create(parent);
    lv_obj_set_size(cont_mode, 200, 60);
    lv_obj_align(cont_mode, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_opa(cont_mode, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cont_mode, 0, 0);
    lv_obj_set_flex_flow(cont_mode, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cont_mode, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    g_alarm_roller_mode = lv_roller_create(cont_mode);
    lv_roller_set_options(g_alarm_roller_mode, "Once\nDaily\nM-F\nWknd", LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(g_alarm_roller_mode, 2);
    lv_obj_set_width(g_alarm_roller_mode, 80);
    lv_roller_set_selected(g_alarm_roller_mode, settings.alarm_mode, LV_ANIM_OFF);
    lv_obj_add_event_cb(g_alarm_roller_mode, event_alarm_save_handler, LV_EVENT_VALUE_CHANGED, NULL);

    g_alarm_sw = lv_switch_create(cont_mode);
    if(settings.alarm_enabled) lv_obj_add_state(g_alarm_sw, LV_STATE_CHECKED);
    lv_obj_add_event_cb(g_alarm_sw, event_alarm_toggle_handler, LV_EVENT_VALUE_CHANGED, NULL);
}

void build_settings_menu(lv_obj_t *parent) {
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(parent, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_t *label = lv_label_create(parent); lv_label_set_text(label, "Settings"); lv_obj_set_style_text_color(label, lv_color_white(), 0); 
    
    lv_obj_t * label_b = lv_label_create(parent); lv_label_set_text(label_b, "Brightness"); lv_obj_set_style_text_color(label_b, lv_color_white(), 0); 
    lv_obj_t * slider = lv_slider_create(parent); lv_obj_set_width(slider, 150); lv_slider_set_range(slider, 10, 255);
    lv_slider_set_value(slider, settings.brightness, LV_ANIM_OFF); lv_obj_add_event_cb(slider, event_brightness_handler, LV_EVENT_ALL, NULL);
    
    lv_obj_t * label_i = lv_label_create(parent); lv_label_set_text(label_i, "IMU Sens."); lv_obj_set_style_text_color(label_i, lv_color_white(), 0); 
    lv_obj_t * slider_i = lv_slider_create(parent); lv_obj_set_width(slider_i, 150); lv_slider_set_range(slider_i, 0, 100);
    lv_slider_set_value(slider_i, settings.imu_sens, LV_ANIM_OFF); lv_obj_add_event_cb(slider_i, event_imu_sens_handler, LV_EVENT_ALL, NULL);
    
    lv_obj_t * label_l = lv_label_create(parent); lv_label_set_text(label_l, "Lift Sens."); lv_obj_set_style_text_color(label_l, lv_color_white(), 0);
    lv_obj_t * slider_l = lv_slider_create(parent); lv_obj_set_width(slider_l, 150); lv_slider_set_range(slider_l, 0, 100);
    lv_slider_set_value(slider_l, settings.lift_sens, LV_ANIM_OFF); lv_obj_add_event_cb(slider_l, event_lift_sens_handler, LV_EVENT_ALL, NULL);
}

void build_activity_screen(lv_obj_t *parent) {
    // Title
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, "Today's Activity");
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);

    // Large progress arc (300 degrees)
    g_activity_arc = lv_arc_create(parent);
    lv_obj_set_size(g_activity_arc, 140, 140);
    lv_obj_center(g_activity_arc);
    lv_arc_set_rotation(g_activity_arc, 120);  // Start at 120° (bottom-left)
    lv_arc_set_bg_angles(g_activity_arc, 0, 300);  // 300° arc
    lv_arc_set_range(g_activity_arc, 0, settings.step_goal);
    
    uint32_t current_steps = settings.steps_today + session_steps;
    lv_arc_set_value(g_activity_arc, current_steps > settings.step_goal ? settings.step_goal : current_steps);
    
    // Arc styling
    lv_obj_set_style_arc_color(g_activity_arc, lv_color_hex(0x00FF00), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(g_activity_arc, 12, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(g_activity_arc, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_arc_width(g_activity_arc, 12, LV_PART_MAIN);
    lv_obj_remove_style(g_activity_arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(g_activity_arc, LV_OBJ_FLAG_CLICKABLE);

    // Center step count (large)
    g_activity_steps_label = lv_label_create(parent);
    char buf[16];
    snprintf(buf, sizeof(buf), "%lu", current_steps);
    lv_label_set_text(g_activity_steps_label, buf);
    lv_obj_set_style_text_font(g_activity_steps_label, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(g_activity_steps_label, lv_color_white(), 0);
    lv_obj_align(g_activity_steps_label, LV_ALIGN_CENTER, 0, -10);

    // Goal label
    g_activity_goal_label = lv_label_create(parent);
    char goal_buf[16];
    snprintf(goal_buf, sizeof(goal_buf), "/ %lu", settings.step_goal);
    lv_label_set_text(g_activity_goal_label, goal_buf);
    lv_obj_set_style_text_font(g_activity_goal_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(g_activity_goal_label, lv_color_hex(0x888888), 0);
    lv_obj_align(g_activity_goal_label, LV_ALIGN_CENTER, 0, 18);

    // Percentage
    g_activity_pct_label = lv_label_create(parent);
    uint8_t pct = (current_steps * 100) / settings.step_goal;
    if (pct > 100) pct = 100;
    snprintf(buf, sizeof(buf), "%u%%", pct);
    lv_label_set_text(g_activity_pct_label, buf);
    lv_obj_set_style_text_font(g_activity_pct_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(g_activity_pct_label, lv_color_hex(0x00FF00), 0);
    lv_obj_align(g_activity_pct_label, LV_ALIGN_CENTER, 0, 37);
}

// NEW: Manual Clock Set Menu (Bottom Tile)
void build_clock_set_menu(lv_obj_t *parent) {
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, "Set Time");
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);

    // Container for Rollers
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, 200, 120);
    lv_obj_align(cont, LV_ALIGN_CENTER, 0, -10);
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Hour Roller
    g_roller_hour = lv_roller_create(cont);
    lv_roller_set_options(g_roller_hour, 
        "00\n01\n02\n03\n04\n05\n06\n07\n08\n09\n10\n11\n12\n13\n14\n15\n16\n17\n18\n19\n20\n21\n22\n23",
        LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(g_roller_hour, 3);
    lv_obj_set_width(g_roller_hour, 60);
    
    lv_obj_t *colon = lv_label_create(cont);
    lv_label_set_text(colon, ":");
    lv_obj_set_style_text_font(colon, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(colon, lv_color_white(), 0);

    // Minute Roller
    g_roller_min = lv_roller_create(cont);
    // Create minute options string 00-59 (Simplified for brevity, usually generated)
    // For manual code, let's just do every 5 mins or full list? Full list is long string in C.
    // I will use a loop to generate string or just static long string. 
    // Static string is safer for embedding.
    lv_roller_set_options(g_roller_min, 
        "00\n01\n02\n03\n04\n05\n06\n07\n08\n09\n10\n11\n12\n13\n14\n15\n16\n17\n18\n19\n"
        "20\n21\n22\n23\n24\n25\n26\n27\n28\n29\n30\n31\n32\n33\n34\n35\n36\n37\n38\n39\n"
        "40\n41\n42\n43\n44\n45\n46\n47\n48\n49\n50\n51\n52\n53\n54\n55\n56\n57\n58\n59",
        LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(g_roller_min, 3);
    lv_obj_set_width(g_roller_min, 60);

    // Set current time to rollers
    lv_roller_set_selected(g_roller_hour, t_now.hour, LV_ANIM_OFF);
    lv_roller_set_selected(g_roller_min, t_now.min, LV_ANIM_OFF);

    // Save Button
    lv_obj_t * btn_save = lv_btn_create(parent);
    lv_obj_set_size(btn_save, 80, 40);
    lv_obj_align(btn_save, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_add_event_cb(btn_save, event_time_save_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn_save, lv_color_hex(0x0000FF), 0);
    
    lv_obj_t * lbl_save = lv_label_create(btn_save);
    lv_label_set_text(lbl_save, "SET");
    lv_obj_center(lbl_save);
}

// Dynamic Tile Manager
static void tile_change_cb(lv_event_t * e) {
    lv_obj_t * tileview = lv_event_get_target(e);
    lv_obj_t * tile = lv_tileview_get_tile_act(tileview);
    
    // Feed watchdog during tile changes
    watchdog_update();
    
    if (tile == tile_center) {
        // Only rebuild if not present
        if (!g_line_hour) build_face_pilot(tile_center);
        else {
            // Update step counter display without rebuilding
            uint32_t current_steps = settings.steps_today + session_steps;
            if (g_step_arc) {
                lv_arc_set_value(g_step_arc, current_steps > settings.step_goal ? settings.step_goal : current_steps);
            }
            if (g_step_label) {
                char buf[16]; snprintf(buf, sizeof(buf), "%lu", current_steps);
                lv_label_set_text(g_step_label, buf);
            }
        }
    } else if (tile == tile_left) {
        if (!g_sw_label_time) build_tools_page(tile_left);
    } else if (tile == tile_right) {
        if (!g_activity_arc) build_activity_screen(tile_right);
        // Update activity display when entering
        if (g_activity_arc) {
            uint32_t current_steps = settings.steps_today + session_steps;
            lv_arc_set_value(g_activity_arc, current_steps > settings.step_goal ? settings.step_goal : current_steps);
            
            if (g_activity_steps_label) {
                char buf[16];
                snprintf(buf, sizeof(buf), "%lu", current_steps);
                lv_label_set_text(g_activity_steps_label, buf);
            }
            
            if (g_activity_pct_label) {
                char buf[8];
                uint8_t pct = (current_steps * 100) / settings.step_goal;
                if (pct > 100) pct = 100;
                snprintf(buf, sizeof(buf), "%u%%", pct);
                lv_label_set_text(g_activity_pct_label, buf);
            }
        }
    } else if (tile == tile_top) {
         if (!lv_obj_get_child_cnt(tile_top)) build_settings_menu(tile_top);
    } else if (tile == tile_bottom) {
         if (!g_roller_hour) build_clock_set_menu(tile_bottom);
    }
}

void setup_ui(void) {
    tv = lv_tileview_create(lv_scr_act());
    static lv_style_t style_black;
    lv_style_init(&style_black);
    lv_style_set_bg_color(&style_black, lv_color_black());
    lv_style_set_bg_opa(&style_black, LV_OPA_COVER);
    lv_obj_add_style(lv_scr_act(), &style_black, 0);
    lv_obj_add_style(tv, &style_black, 0);
    
    tile_center = lv_tileview_add_tile(tv, 1, 1, LV_DIR_ALL); 
    tile_left = lv_tileview_add_tile(tv, 0, 1, LV_DIR_RIGHT);
    tile_right = lv_tileview_add_tile(tv, 2, 1, LV_DIR_LEFT);
    tile_top = lv_tileview_add_tile(tv, 1, 0, LV_DIR_BOTTOM);
    tile_bottom = lv_tileview_add_tile(tv, 1, 2, LV_DIR_TOP);
    
    // BUILD ALL TILES AT STARTUP (Static Loading Strategy - Stable V2.0)
    build_face_pilot(tile_center); 
    build_tools_page(tile_left); 
    build_activity_screen(tile_right); 
    build_settings_menu(tile_top); 
    build_clock_set_menu(tile_bottom); // Swapped System for Clock Set
    
    lv_obj_set_tile_id(tv, 1, 1, LV_ANIM_OFF);
    
    // Re-enable dynamic loading for better memory management
    lv_obj_add_event_cb(tv, tile_change_cb, LV_EVENT_VALUE_CHANGED, NULL);
}

int main() {
    set_sys_clock_khz(133000, true); 
    stdio_init_all();
    sleep_ms(100); // Reduced from 2000ms

    spi_init(LCD_SPI_PORT, 20 * 1000 * 1000); // Reduced from 40MHz for power savings 
    gpio_set_function(LCD_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LCD_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_init(LCD_RST_PIN); gpio_set_dir(LCD_RST_PIN, GPIO_OUT);
    gpio_init(LCD_DC_PIN);  gpio_set_dir(LCD_DC_PIN, GPIO_OUT);
    gpio_init(LCD_CS_PIN);  gpio_set_dir(LCD_CS_PIN, GPIO_OUT);
    gpio_put(LCD_CS_PIN, 1); gpio_put(LCD_DC_PIN, 1); 
    
    gpio_init(TOUCH_RST_PIN); gpio_set_dir(TOUCH_RST_PIN, GPIO_OUT);
    gpio_put(TOUCH_RST_PIN, 1); sleep_ms(10); gpio_put(TOUCH_RST_PIN, 0); sleep_ms(10); gpio_put(TOUCH_RST_PIN, 1); sleep_ms(50);

    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C); // Fixed SCL
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    gpio_set_function(LCD_BL_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(LCD_BL_PIN);
    pwm_set_wrap(slice_num, 65535); 
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 65535); 
    pwm_set_enabled(slice_num, true);

    lcd_init(); 
    
    // IMU Init
    uint8_t dummy;
    if (i2c_read_blocking(I2C_PORT, TOUCH_ADDR, &dummy, 1, false) > 0) touch_found = true;
    
    if (i2c_write_blocking(I2C_PORT, 0x6B, &dummy, 0, false) != PICO_ERROR_GENERIC) {
        detected_imu_addr = 0x6B;
        imu_found = true;
    } else if (i2c_write_blocking(I2C_PORT, 0x6A, &dummy, 0, false) != PICO_ERROR_GENERIC) {
        detected_imu_addr = 0x6A;
        imu_found = true;
    }
    
    if (imu_found) {
        uint8_t cmd_reset[2] = {0x60, 0xB6}; 
        i2c_write_blocking(I2C_PORT, detected_imu_addr, cmd_reset, 2, false);
        sleep_ms(20); 
        uint8_t cmd_ctrl7[2] = {0x08, 0x03}; 
        i2c_write_blocking(I2C_PORT, detected_imu_addr, cmd_ctrl7, 2, false);
        uint8_t cmd_ctrl2[2] = {0x03, 0x23}; 
        i2c_write_blocking(I2C_PORT, detected_imu_addr, cmd_ctrl2, 2, false);
        uint8_t cmd_ctrl1[2] = {0x02, 0x60}; 
        i2c_write_blocking(I2C_PORT, detected_imu_addr, cmd_ctrl1, 2, false);
    } 

    adc_init();
    adc_gpio_init(BATTERY_PIN);
    adc_select_input(3);
    // Disable ADC initially to save power - enabled only when reading
    hw_clear_bits(&adc_hw->cs, ADC_CS_EN_BITS);
    
    // Buzzer Init
    init_buzzer();

    watchdog_enable(8000, 1);

    load_settings();

    lv_init();
    lvgl_init_drivers();
    setup_ui(); 
    init_lvgl_tick();

    // Init Clock with Default Time (Jan 1 2025) so it ticks immediately
    rtc_init();
    datetime_t t_default = { .year = 2025, .month = 1, .day = 1, .dotw = 3, .hour = 12, .min = 0, .sec = 0 };
    rtc_set_datetime(&t_default);

    last_activity_time = to_ms_since_boot(get_absolute_time());
    
    // Set initial IMU power mode if found
    if (imu_found) {
        imu_set_low_power_mode(false); // Start in normal mode
    }
    
    // Startup Chirp - 6.3kHz matches buzzer resonant frequency
    play_tone(6300, 100);
    
    while (true) {
        loop_start_time = to_ms_since_boot(get_absolute_time()); // Measure load start
        watchdog_update();
        uint32_t now_ms = loop_start_time;
        
        // Skip USB polling in sleep mode
        if (pwr_state != PWR_SLEEP) {
            check_usb_sync();
        }
        rtc_get_datetime(&t_now);
        
        uint32_t idle_time = now_ms - last_activity_time;
        if (pwr_state == PWR_ACTIVE && idle_time > TIMEOUT_IDLE_MS) set_power_state(PWR_IDLE);
        else if (pwr_state == PWR_IDLE && idle_time > TIMEOUT_SLEEP_MS) set_power_state(PWR_SLEEP);

        if (imu_found) {
            // Reduce IMU polling in sleep mode for power savings
            if (pwr_state == PWR_SLEEP) {
                if (now_ms - last_imu_read_sleep > 100) { // Only read every 100ms in sleep
                    read_imu();
                    check_lift_to_wake();
                    last_imu_read_sleep = now_ms;
                }
            } else {
                read_imu();
                check_lift_to_wake();
                update_step_counter();
            }
        }
        
        // Only update UI when not sleeping
        if (pwr_state != PWR_SLEEP) {
            update_clock_ui(now_ms); 
            update_timer_logic();
            update_stopwatch_logic(); 
            update_diagnostics_logic();
            check_alarm_trigger();
        }
        
        check_buzzer_state(now_ms);
        
        // Only run LVGL handler when not sleeping
        if (pwr_state != PWR_SLEEP) {
            lv_timer_handler();
        } 

        // Measure work time for load calculation
        uint32_t work_time_us = to_us_since_boot(get_absolute_time()) - (loop_start_time * 1000);
        
        if (pwr_state == PWR_ACTIVE) {
            sleep_ms(5); // Increased from 1ms - still responsive but saves power
            cpu_load_pct = (work_time_us * 100) / (work_time_us + 5000);
        }
        else if (pwr_state == PWR_IDLE) {
            sleep_ms(10); // Slower updates in idle
            cpu_load_pct = (work_time_us * 100) / (work_time_us + 10000);
        }
        else if (pwr_state == PWR_SLEEP) {
            sleep_ms(200); // Increased from 100ms - only need to check for wake
            cpu_load_pct = 0;
        }
        
        if (now_ms - last_bat_read > 5000) { // Increased from 3000ms
            read_battery();
            last_bat_read = now_ms;
            
            if (t_now.day != settings.last_day_saved && settings.last_day_saved != 0) {
                 save_settings();
            }
        }
        
        // Debounced settings save (500ms after last change)
        if (settings_needs_save && (now_ms - settings_changed_time > 500)) {
            save_settings();
            settings_needs_save = false;
        }
    }
}