// ======================= PhotonNexus Mk.1 Complete - PIC18F46K22 ================
// Toolchain: MPLAB X + XC8
// Clock: Internal 16 MHz
// Servos: RC2 (azimuth), RB3 (tilt) - OPTIMIZED TRACKING ALGORITHM
// LDRs: AN0=TopLeft, AN1=TopRight, AN2=BottomLeft, AN3=BottomRight
// I2C: INA219 on SDA1/SCL1 (RC4/RC3) for solar panel monitoring
// LCD: 4-bit HD44780 (RD0=RS, RD1=E, RD4..RD7=D4..D7)
// Features: Auto-cycling display, battery %, power monitoring, daily energy
//          Adaptive servo control, filtered LDR readings, return to center
// ==============================================================================

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define _XTAL_FREQ 16000000UL

// Interrupt macros
#define di() do { INTCONbits.GIE = 0; } while(0)
#define ei() do { INTCONbits.PEIE = 1; INTCONbits.GIE = 1; } while(0)

// ====================== CONFIG BITS ===========================================
#pragma config FOSC = INTIO67
#pragma config PLLCFG = OFF
#pragma config PRICLKEN = ON
#pragma config FCMEN = OFF, IESO = OFF
#pragma config PWRTEN = OFF, BOREN = SBORDIS, BORV = 285
#pragma config WDTEN = OFF, WDTPS = 32768
#pragma config CCP2MX = PORTB3
#pragma config PBADEN = OFF
#pragma config LVP = ON
#pragma config MCLRE = EXTMCLR

// ========================= PIN MAP ============================================
#define SERVO_AZ_LAT    LATCbits.LATC2
#define SERVO_AZ_TRIS   TRISCbits.TRISC2
#define SERVO_TILT_LAT  LATBbits.LATB3
#define SERVO_TILT_TRIS TRISBbits.TRISB3

// LCD (4-bit)
#define LCD_RS_LAT     LATDbits.LATD0
#define LCD_E_LAT      LATDbits.LATD1
#define LCD_D4_LAT     LATDbits.LATD4
#define LCD_D5_LAT     LATDbits.LATD5
#define LCD_D6_LAT     LATDbits.LATD6
#define LCD_D7_LAT     LATDbits.LATD7
#define LCD_RS_TRIS    TRISDbits.TRISD0
#define LCD_E_TRIS     TRISDbits.TRISD1
#define LCD_D4_TRIS    TRISDbits.TRISD4
#define LCD_D5_TRIS    TRISDbits.TRISD5
#define LCD_D6_TRIS    TRISDbits.TRISD6
#define LCD_D7_TRIS    TRISDbits.TRISD7

// ========================== SERVO CONSTANTS ===================================
// Servo limits - ADJUST THESE TO YOUR HARDWARE
#define SERVO_AZ_MIN        600
#define SERVO_AZ_MAX        2400
#define SERVO_TILT_MIN      600
#define SERVO_TILT_MAX      2400
#define SERVO_US_CENTER     1500

// Adaptive step sizing for smoother control
#define SERVO_STEP_MAX      40      // Maximum step when far from target
#define SERVO_STEP_MIN      10      // Minimum step when close to target
#define SERVO_DEADBAND      30      // Deadband for better precision

// Proportional control gain
#define PROPORTIONAL_GAIN   15      // Higher = more aggressive tracking

// Light detection thresholds
#define LIGHT_LOST_THRESHOLD 400    // Total light threshold
#define MIN_VALID_READING    50     // Individual LDR minimum

// Low-pass filter coefficient (0-255, higher = more smoothing)
#define FILTER_ALPHA        200

// ========================== DISPLAY CONSTANTS =================================
#define PAGE_DISPLAY_MS     5000    // 5 seconds per page
#define NUM_PAGES           5       // Total display pages

// ========================== LDR CHANNELS ======================================
#define LDR_TL 0  // Top-Left
#define LDR_TR 1  // Top-Right
#define LDR_BL 2  // Bottom-Left
#define LDR_BR 3  // Bottom-Right

// ========================== INA219 CONSTANTS ==================================
#define INA219_ADDR           0x40
#define INA219_REG_BUSVOLTAGE 0x02
#define INA219_REG_CURRENT    0x04
#define INA219_REG_POWER      0x03

// Battery parameters
#define BATTERY_MIN_MV        3000  // 3.0V empty
#define BATTERY_MAX_MV        4200  // 4.2V full
#define BATTERY_CAPACITY_MAH  6600  // Battery capacity in mAh (adjust to your battery)

// ========================== GLOBAL STATE ======================================
// Servo PWM state
volatile uint16_t pulse_az_us = SERVO_US_CENTER;
volatile uint16_t pulse_tilt_us = SERVO_US_CENTER;
volatile uint8_t pwm_phase = 0;

// Millis counter
static volatile uint32_t millis = 0;

// LDR readings
uint16_t ldr_values[4] = {0};
uint16_t ldr_filtered[4] = {0};
uint8_t filter_initialized = 0;

// Power monitoring
static uint16_t battery_mv = 3700;
static uint16_t solar_mv = 0;
static int16_t  solar_ma = 0;
static uint16_t solar_mw = 0;
static uint32_t daily_energy_mwh = 0;
static uint32_t last_energy_update = 0;
static uint16_t time_to_full_min = 0;  // Time to full charge in minutes

// Display state
static uint8_t current_page = 0;
static uint32_t last_page_change = 0;

// Light tracking state
static uint8_t light_lost_count = 0;

// ========================== UTILITY FUNCTIONS =================================
static inline int16_t abs16(int16_t x) {
    return (x < 0) ? -x : x;
}

static inline uint16_t constrain_u16(uint16_t val, uint16_t min, uint16_t max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

// Calculate adaptive step size based on error magnitude
static uint16_t calc_adaptive_step(int16_t error) {
    int16_t abs_error = abs16(error);
    
    // Proportional step: larger errors get bigger steps
    uint16_t step = (uint16_t)(abs_error / PROPORTIONAL_GAIN);
    
    // Constrain step size
    if (step < SERVO_STEP_MIN) step = SERVO_STEP_MIN;
    if (step > SERVO_STEP_MAX) step = SERVO_STEP_MAX;
    
    return step;
}

// ========================== TIMER0: MILLIS ====================================
void timer0_init(void){
    T0CON = 0b11000100;
    TMR0 = 131;
    INTCONbits.TMR0IF = 0;
    INTCONbits.TMR0IE = 1;
}

// ========================== TIMER1: SERVO PWM =================================
static void timer1_init(void) {
    T1CON = 0b00110001;
    TMR1 = 0;
    PIR1bits.TMR1IF = 0;
    PIE1bits.TMR1IE = 1;
}

// ========================== INTERRUPT SERVICE ROUTINE =========================
void __interrupt() isr(void){
    // TMR0: millis counter
    if (INTCONbits.TMR0IE && INTCONbits.TMR0IF){
        INTCONbits.TMR0IF = 0;
        TMR0 += 131;
        millis++;
    }
    
    // TMR1: optimized servo PWM generation
    if (PIE1bits.TMR1IE && PIR1bits.TMR1IF) {
        PIR1bits.TMR1IF = 0;
        
        // Cache volatile reads at start of ISR
        static uint16_t cached_az, cached_tilt;
        
        switch (pwm_phase) {
            case 0: {
                // Start new pulse cycle
                cached_az = pulse_az_us;
                cached_tilt = pulse_tilt_us;
                
                SERVO_AZ_LAT = 1;
                SERVO_TILT_LAT = 1;
                
                uint16_t first = (cached_az <= cached_tilt) ? cached_az : cached_tilt;
                uint16_t ticks = first >> 1;  // Divide by 2 (bit shift is faster)
                uint16_t preload = 65536 - ticks;
                
                TMR1H = preload >> 8;
                TMR1L = (uint8_t)preload;
                pwm_phase = (cached_az <= cached_tilt) ? 1 : 2;
            } break;

            case 1: {
                SERVO_AZ_LAT = 0;
                
                if (cached_tilt > cached_az) {
                    uint16_t ticks = (cached_tilt - cached_az) >> 1;
                    uint16_t preload = 65536 - ticks;
                    TMR1H = preload >> 8;
                    TMR1L = (uint8_t)preload;
                    pwm_phase = 3;
                } else {
                    uint16_t top = (cached_az > cached_tilt) ? cached_az : cached_tilt;
                    uint16_t ticks = (20000 - top) >> 1;
                    uint16_t preload = 65536 - ticks;
                    TMR1H = preload >> 8;
                    TMR1L = (uint8_t)preload;
                    pwm_phase = 4;
                }
            } break;

            case 2: {
                SERVO_TILT_LAT = 0;
                
                if (cached_az > cached_tilt) {
                    uint16_t ticks = (cached_az - cached_tilt) >> 1;
                    uint16_t preload = 65536 - ticks;
                    TMR1H = preload >> 8;
                    TMR1L = (uint8_t)preload;
                    pwm_phase = 3;
                } else {
                    uint16_t top = (cached_az > cached_tilt) ? cached_az : cached_tilt;
                    uint16_t ticks = (20000 - top) >> 1;
                    uint16_t preload = 65536 - ticks;
                    TMR1H = preload >> 8;
                    TMR1L = (uint8_t)preload;
                    pwm_phase = 4;
                }
            } break;

            case 3: {
                SERVO_AZ_LAT = 0;
                SERVO_TILT_LAT = 0;
                
                uint16_t top = (cached_az > cached_tilt) ? cached_az : cached_tilt;
                uint16_t ticks = (20000 - top) >> 1;
                uint16_t preload = 65536 - ticks;
                
                TMR1H = preload >> 8;
                TMR1L = (uint8_t)preload;
                pwm_phase = 4;
            } break;

            case 4: {
                pwm_phase = 0;
            } break;
        }
    }
}

// ========================== CLOCK / GPIO / ADC ================================
static void clock_init(void){
    OSCCONbits.IRCF = 0b111;
    OSCCONbits.SCS  = 0b10;
    while(!OSCCONbits.HFIOFS);
}

static void gpio_init(void){
    ANSELA = 0x0F;  // AN0-AN3 analog for LDRs
    TRISA |= 0x0F;
    ANSELB = 0x00;
    ANSELC = 0x00;
    ANSELD = 0x00;
    ANSELE = 0x00;

    SERVO_AZ_TRIS = 0; SERVO_AZ_LAT = 0;
    SERVO_TILT_TRIS = 0; SERVO_TILT_LAT = 0;

    LCD_RS_TRIS = 0; LCD_E_TRIS = 0;
    LCD_D4_TRIS = 0; LCD_D5_TRIS = 0; LCD_D6_TRIS = 0; LCD_D7_TRIS = 0;
    LCD_RS_LAT=0; LCD_E_LAT=0; LCD_D4_LAT=0; LCD_D5_LAT=0; LCD_D6_LAT=0; LCD_D7_LAT=0;
}

static void adc_init(void){
    ADCON0 = 0x00;
    ADCON1 = 0x00;
    ADCON2 = 0b10101110;  // Right justified, 12 TAD, FOSC/64
    ADCON0bits.ADON = 1;
}

static uint16_t adc_read(uint8_t channel) {
    ADCON0bits.CHS = channel;
    ADCON0bits.ADON = 1;
    __delay_us(10);
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO);
    return (uint16_t)((ADRESH << 8) | ADRESL);
}

// Low-pass filter for ADC readings (reduces noise)
static uint16_t adc_read_filtered(uint8_t channel) {
    uint16_t raw = adc_read(channel);
    
    if (!filter_initialized) {
        ldr_filtered[channel] = raw;
        return raw;
    }
    
    // Exponential moving average: filtered = (alpha * raw + (256-alpha) * old) / 256
    uint32_t temp = ((uint32_t)FILTER_ALPHA * raw) + 
                    ((uint32_t)(256 - FILTER_ALPHA) * ldr_filtered[channel]);
    ldr_filtered[channel] = (uint16_t)(temp >> 8);
    
    return ldr_filtered[channel];
}

// ========================== I2C FUNCTIONS =====================================
static void i2c_init(void){
    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC4 = 1;
    SSPCON1 = 0b00101000;
    SSPCON2 = 0x00;
    SSPADD = 39;  // 100kHz @ 16MHz
    SSPSTAT = 0b10000000;
}

static void i2c_wait(void){
    while((SSPCON2 & 0x1F) || (SSPSTATbits.R_W));
}

static void i2c_start(void){
    i2c_wait();
    SSPCON2bits.SEN = 1;
}

static void i2c_stop(void){
    i2c_wait();
    SSPCON2bits.PEN = 1;
}

static void i2c_write(uint8_t data){
    i2c_wait();
    SSPBUF = data;
}

static uint8_t i2c_read(uint8_t ack){
    i2c_wait();
    SSPCON2bits.RCEN = 1;
    i2c_wait();
    uint8_t data = SSPBUF;
    i2c_wait();
    if(ack) SSPCON2bits.ACKDT = 0;
    else SSPCON2bits.ACKDT = 1;
    SSPCON2bits.ACKEN = 1;
    return data;
}

static int16_t ina219_read_reg(uint8_t reg){
    i2c_start();
    i2c_write((INA219_ADDR << 1) | 0);
    i2c_write(reg);
    i2c_start();
    i2c_write((INA219_ADDR << 1) | 1);
    uint8_t msb = i2c_read(1);
    uint8_t lsb = i2c_read(0);
    i2c_stop();
    return (int16_t)((msb << 8) | lsb);
}

static void update_power_readings(void){
    // Read bus voltage (solar panel voltage)
    int16_t bus_raw = ina219_read_reg(INA219_REG_BUSVOLTAGE);
    solar_mv = (uint16_t)((bus_raw >> 3) * 4);  // 4mV per LSB
    
    // Read current
    int16_t current_raw = ina219_read_reg(INA219_REG_CURRENT);
    solar_ma = current_raw;
    
    // Calculate power
    solar_mw = (uint16_t)((solar_mv * (uint32_t)solar_ma) / 1000);
    
    // Placeholder for battery voltage - add actual monitoring via ADC or second INA219
    battery_mv = 3700;
    
    // Calculate time to full charge
    if (solar_ma > 0 && battery_mv < BATTERY_MAX_MV) {
        // Calculate current battery percentage
        int16_t batt_pct;
        if (battery_mv >= BATTERY_MAX_MV) {
            batt_pct = 100;
        } else if (battery_mv <= BATTERY_MIN_MV) {
            batt_pct = 0;
        } else {
            batt_pct = (int16_t)(((uint32_t)(battery_mv - BATTERY_MIN_MV) * 100) / 
                                 (BATTERY_MAX_MV - BATTERY_MIN_MV));
        }
        
        // Calculate remaining capacity needed
        uint32_t remaining_mah = (uint32_t)((100 - batt_pct) * BATTERY_CAPACITY_MAH) / 100;
        
        // Calculate time to full: remaining_mah / charging_current_ma
        // Time in hours = mAh / mA, convert to minutes
        if (solar_ma > 10) {  // Only calculate if charging current is significant
            time_to_full_min = (uint16_t)((remaining_mah * 60) / solar_ma);
        } else {
            time_to_full_min = 9999;  // Display "---" for very low current
        }
    } else {
        time_to_full_min = 0;  // Already full or not charging
    }
}

static void update_daily_energy(void){
    uint32_t now = millis;
    if(last_energy_update == 0){
        last_energy_update = now;
        return;
    }
    
    uint32_t elapsed_ms = now - last_energy_update;
    daily_energy_mwh += (solar_mw * elapsed_ms) / 3600000;
    last_energy_update = now;
}

// ========================== LCD FUNCTIONS =====================================
static void lcd_pulse(void){ 
    LCD_E_LAT=1; 
    __delay_us(2); 
    LCD_E_LAT=0; 
    __delay_us(50); 
}

static void lcd_send4(uint8_t n){ 
    LCD_D4_LAT=(n>>0)&1; 
    LCD_D5_LAT=(n>>1)&1; 
    LCD_D6_LAT=(n>>2)&1; 
    LCD_D7_LAT=(n>>3)&1; 
    lcd_pulse(); 
}

static void lcd_cmd(uint8_t c){ 
    LCD_RS_LAT=0; 
    lcd_send4(c>>4); 
    lcd_send4(c&0xF); 
    if(c==0x01||c==0x02) __delay_ms(2); 
    else __delay_us(50);
}

static void lcd_dat(uint8_t d){ 
    LCD_RS_LAT=1; 
    lcd_send4(d>>4); 
    lcd_send4(d&0xF); 
    __delay_us(50);
}

static void lcd_init(void){
    __delay_ms(50);
    LCD_RS_LAT=0; LCD_E_LAT=0;
    lcd_send4(0x03); __delay_ms(5);
    lcd_send4(0x03); __delay_ms(1);
    lcd_send4(0x03); __delay_ms(1);
    lcd_send4(0x02); __delay_ms(1);
    lcd_cmd(0x28);
    lcd_cmd(0x0C);
    lcd_cmd(0x06);
    lcd_cmd(0x01);
    __delay_ms(2);
}

static void lcd_setxy(uint8_t col, uint8_t row){ 
    lcd_cmd(0x80 + (row?0x40:0x00) + col); 
}

static void lcd_print(const char* s){ 
    while(*s) lcd_dat(*s++); 
}

// ========================== DISPLAY PAGES =====================================
static void display_page_0(void){
    // Boot banner / System info
    lcd_cmd(0x01);
    lcd_setxy(0,0);
    lcd_print("PhotonNexus Mk.1");
    lcd_setxy(0,1);
    lcd_print("v2.0 Enhanced");
}

static void display_page_1(void){
    char buf[17];
    lcd_cmd(0x01);
    lcd_setxy(0,0);
    lcd_print("Photon Intake");
    lcd_setxy(0,1);
    sprintf(buf, "%u.%uV %dmA", solar_mv/1000, (solar_mv%1000)/100, solar_ma);
    lcd_print(buf);
}

static void display_page_2(void){
    char buf[17];
    lcd_cmd(0x01);
    lcd_setxy(0,0);
    lcd_print("Battery Status");
    
    int16_t batt_pct = 0;
    if(battery_mv >= BATTERY_MAX_MV){
        batt_pct = 100;
    } else if(battery_mv <= BATTERY_MIN_MV){
        batt_pct = 0;
    } else {
        batt_pct = (int16_t)(((uint32_t)(battery_mv - BATTERY_MIN_MV) * 100) / 
                             (BATTERY_MAX_MV - BATTERY_MIN_MV));
    }
    
    lcd_setxy(0,1);
    sprintf(buf, "%u.%uV  %d%%", battery_mv/1000, (battery_mv%1000)/100, batt_pct);
    lcd_print(buf);
}

static void display_page_3(void){
    char buf[17];
    lcd_cmd(0x01);
    lcd_setxy(0,0);
    lcd_print("Time to Full");
    lcd_setxy(0,1);
    
    if (battery_mv >= BATTERY_MAX_MV) {
        lcd_print("Fully Charged!");
    } else if (time_to_full_min == 0 || solar_ma <= 0) {
        lcd_print("Not Charging");
    } else if (time_to_full_min >= 9999) {
        lcd_print("--- (Low Power)");
    } else if (time_to_full_min >= 1440) {  // More than 24 hours
        uint16_t days = time_to_full_min / 1440;
        uint16_t hours = (time_to_full_min % 1440) / 60;
        sprintf(buf, "%ud %uh", days, hours);
        lcd_print(buf);
    } else if (time_to_full_min >= 60) {
        uint16_t hours = time_to_full_min / 60;
        uint16_t mins = time_to_full_min % 60;
        sprintf(buf, "%uh %umin", hours, mins);
        lcd_print(buf);
    } else {
        sprintf(buf, "%u minutes", time_to_full_min);
        lcd_print(buf);
    }
}

static void display_page_4(void){
    char buf[17];
    lcd_cmd(0x01);
    lcd_setxy(0,0);
    lcd_print("Daily Energy");
    lcd_setxy(0,1);
    uint16_t wh = (uint16_t)(daily_energy_mwh / 1000);
    uint16_t mwh_frac = (uint16_t)(daily_energy_mwh % 1000);
    sprintf(buf, "%u.%03u Watts/hr", wh, mwh_frac);
    lcd_print(buf);
}

static void update_display(void){
    uint32_t now;
    di();
    now = millis;
    ei();
    
    // Auto-cycle pages every 5 seconds
    if(now - last_page_change >= PAGE_DISPLAY_MS){
        current_page = (current_page + 1) % NUM_PAGES;
        last_page_change = now;
    }
    
    // Display current page
    switch(current_page){
        case 0: display_page_0(); break;
        case 1: display_page_1(); break;
        case 2: display_page_2(); break;
        case 3: display_page_3(); break;
        case 4: display_page_4(); break;
    }
}

// ========================== LIGHT VALIDATION ==================================
static uint8_t validate_light_readings(void) {
    uint16_t total = 0;
    uint8_t valid_count = 0;
    
    for (uint8_t i = 0; i < 4; i++) {
        if (ldr_values[i] > MIN_VALID_READING) {
            valid_count++;
            total += ldr_values[i];
        }
    }
    
    return (valid_count >= 2 && total > LIGHT_LOST_THRESHOLD);
}

// ========================== RETURN TO CENTER ==================================
static void return_to_center(void) {
    while (pulse_az_us != SERVO_US_CENTER || pulse_tilt_us != SERVO_US_CENTER) {
        int16_t az_error = (int16_t)SERVO_US_CENTER - (int16_t)pulse_az_us;
        int16_t tilt_error = (int16_t)SERVO_US_CENTER - (int16_t)pulse_tilt_us;
        
        uint16_t az_step = calc_adaptive_step(az_error);
        uint16_t tilt_step = calc_adaptive_step(tilt_error);
        
        if (abs16(az_error) <= az_step) {
            pulse_az_us = SERVO_US_CENTER;
        } else if (az_error > 0) {
            pulse_az_us += az_step;
        } else {
            pulse_az_us -= az_step;
        }
        
        if (abs16(tilt_error) <= tilt_step) {
            pulse_tilt_us = SERVO_US_CENTER;
        } else if (tilt_error > 0) {
            pulse_tilt_us += tilt_step;
        } else {
            pulse_tilt_us -= tilt_step;
        }
        
        __delay_ms(20);
    }
}

// ========================== MAIN ==============================================
void main(void){
    di();
    clock_init();
    gpio_init();
    adc_init();
    lcd_init();
    i2c_init();
    timer0_init();
    timer1_init();
    ei();

    // Boot banner
    lcd_setxy(0,0); lcd_print("PhotonNexus Mk.1");
    lcd_setxy(0,1); lcd_print("v2.0 Enhanced");
    __delay_ms(2000);

    // Center servos
    pulse_az_us = SERVO_US_CENTER;
    pulse_tilt_us = SERVO_US_CENTER;
    __delay_ms(1000);

    // Initialize filter values
    for (uint8_t i = 0; i < 4; i++) {
        ldr_filtered[i] = adc_read(i);
    }
    filter_initialized = 1;

    // Initialize display timing
    di();
    last_page_change = millis;
    last_energy_update = millis;
    ei();

    // ===================== MAIN TRACKING LOOP =================================
    uint32_t last_power_update = 0;

    while(1){
        uint32_t now;
        di();
        now = millis;
        ei();
        
        // Update power readings every 500ms
        if(now - last_power_update >= 500){
            update_power_readings();
            update_daily_energy();
            last_power_update = now;
        }
        
        // Update display (auto-cycles internally)
        update_display();
        
        // Read and filter all 4 LDRs
        for (uint8_t i = 0; i < 4; i++) {
            ldr_values[i] = adc_read_filtered(i);
        }

        // Validate light readings with debouncing
        if (!validate_light_readings()) {
            light_lost_count++;
            if (light_lost_count > 3) {
                return_to_center();
                light_lost_count = 0;
                __delay_ms(500);
                continue;
            }
        } else {
            light_lost_count = 0;
        }

        // Calculate averages for each axis
        int16_t left_avg = (ldr_values[LDR_TL] + ldr_values[LDR_BL]) >> 1;
        int16_t right_avg = (ldr_values[LDR_TR] + ldr_values[LDR_BR]) >> 1;
        int16_t horiz_diff = left_avg - right_avg;

        int16_t top_avg = (ldr_values[LDR_TL] + ldr_values[LDR_TR]) >> 1;
        int16_t bot_avg = (ldr_values[LDR_BL] + ldr_values[LDR_BR]) >> 1;
        int16_t vert_diff = top_avg - bot_avg;

        // SIMULTANEOUS AZIMUTH AND TILT CONTROL with adaptive stepping
        bool moved = false;
        
        // AZIMUTH CONTROL
        if (abs16(horiz_diff) > SERVO_DEADBAND) {
            uint16_t step = calc_adaptive_step(horiz_diff);
            
            if (horiz_diff > 0) {
                pulse_az_us = constrain_u16(pulse_az_us + step, 
                                           SERVO_AZ_MIN, SERVO_AZ_MAX);
            } else {
                pulse_az_us = constrain_u16(pulse_az_us - step, 
                                           SERVO_AZ_MIN, SERVO_AZ_MAX);
            }
            moved = true;
        }

        // TILT CONTROL
        if (abs16(vert_diff) > SERVO_DEADBAND) {
            uint16_t step = calc_adaptive_step(vert_diff);
            
            if (vert_diff > 0) {
                pulse_tilt_us = constrain_u16(pulse_tilt_us + step,
                        SERVO_TILT_MIN, SERVO_TILT_MAX);
            } else {
                pulse_tilt_us = constrain_u16(pulse_tilt_us - step,
                        SERVO_TILT_MIN, SERVO_TILT_MAX);
            }
            moved = true;
        }

        __delay_ms(25);
    }

}
