/**
 * @file    ds1337.h
 * @brief   Hardware-agnostic driver for Maxim/Dallas DS1337 I2C Real-Time Clock.
 * @note    This driver manages BCD conversions and Alarm Mask configurations internally.
 */

#ifndef DS1337_H
#define DS1337_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================= */
/* ==================== CONFIGURATION MACROS =============================== */
/* ========================================================================= */

#ifndef DS1337_CFG_TIMEOUT
#define DS1337_CFG_TIMEOUT (10U) /**< I2C transaction timeout */
#endif

#define DS1337_I2C_ADDR 0x68 /**< Fixed 7-bit I2C address */

/* Registers */
#define DS1337_REG_SECONDS 0x00
#define DS1337_REG_MINUTES 0x01
#define DS1337_REG_HOURS   0x02
#define DS1337_REG_DAY     0x03
#define DS1337_REG_DATE    0x04
#define DS1337_REG_MONTH   0x05 /**< Also contains Century bit (Bit 7) */
#define DS1337_REG_YEAR    0x06

#define DS1337_REG_ALARM1_SEC  0x07
#define DS1337_REG_ALARM1_MIN  0x08
#define DS1337_REG_ALARM1_HRS  0x09
#define DS1337_REG_ALARM1_DATE 0x0A

#define DS1337_REG_ALARM2_MIN  0x0B
#define DS1337_REG_ALARM2_HRS  0x0C
#define DS1337_REG_ALARM2_DATE 0x0D

#define DS1337_REG_CONTROL 0x0E
#define DS1337_REG_STATUS  0x0F

/* ========================================================================= */
/* ==================== ENUMS & DATA STRUCTURES ============================ */
/* ========================================================================= */

/**
 * @brief Standard Decimal Time Structure
 */
typedef struct
{
    uint8_t year;    /**< 00 to 99 */
    uint8_t month;   /**< 01 to 12 */
    uint8_t date;    /**< 01 to 31 */
    uint8_t day;     /**< 01 to 07 (1 = Sunday commonly) */
    uint8_t hours;   /**< 00 to 23 (Driver forces 24-hour mode) */
    uint8_t minutes; /**< 00 to 59 */
    uint8_t seconds; /**< 00 to 59 */
} ds1337_time_t;

/**
 * @brief Alarm 1 Trigger Rate Configuration
 */
typedef enum
{
    DS1337_A1_EVERY_SECOND = 0x0F,     /**< Alarm once per second */
    DS1337_A1_MATCH_SEC = 0x0E,        /**< Alarm when seconds match */
    DS1337_A1_MATCH_MIN_SEC = 0x0C,    /**< Alarm when minutes and seconds match */
    DS1337_A1_MATCH_HR_MIN_SEC = 0x08, /**< Alarm when hours, minutes, and seconds match */
    DS1337_A1_MATCH_DATE = 0x00,       /**< Alarm when date, hours, minutes, seconds match */
    DS1337_A1_MATCH_DAY = 0x40         /**< Alarm when day, hours, minutes, seconds match */
} ds1337_alarm1_rate_et;

/**
 * @brief Alarm 2 Trigger Rate Configuration
 */
typedef enum
{
    DS1337_A2_EVERY_MINUTE = 0x07, /**< Alarm once per minute (at 00 seconds) */
    DS1337_A2_MATCH_MIN = 0x06,    /**< Alarm when minutes match */
    DS1337_A2_MATCH_HR_MIN = 0x04, /**< Alarm when hours and minutes match */
    DS1337_A2_MATCH_DATE = 0x00,   /**< Alarm when date, hours, minutes match */
    DS1337_A2_MATCH_DAY = 0x40     /**< Alarm when day, hours, minutes match */
} ds1337_alarm2_rate_et;

/**
 * @brief Square-Wave / Interrupt output configuration (INTCN)
 */
typedef enum
{
    DS1337_INTCN_SQW = 0,  /**< Output Square Wave on INTA */
    DS1337_INTCN_ALARM = 1 /**< Output Alarm Interrupts on INTA/INTB */
} ds1337_int_mode_et;

/**
 * @brief Status Flag Bit-masks
 */
#define DS1337_STATUS_A1F 0x01 /**< Alarm 1 Flag */
#define DS1337_STATUS_A2F 0x02 /**< Alarm 2 Flag */
#define DS1337_STATUS_OSF 0x80 /**< Oscillator Stop Flag */

/**
 * @brief Initialization Configuration
 */
typedef struct
{
    ds1337_int_mode_et int_mode;
    bool enable_oscillator; /**< Should be true to keep time */
} ds1337_init_t;

/**
 * @brief Runtime Device Object
 */
typedef struct
{
    iic_halt bus;
    addr16_gt dev_addr;
} ds1337_dev_t;

/* ========================================================================= */
/* ==================== API FUNCTIONS ====================================== */
/* ========================================================================= */

ec_gt ds1337_init(ds1337_dev_t* dev, iic_halt bus, const ds1337_init_t* init_cfg);

ec_gt ds1337_set_time(ds1337_dev_t* dev, const ds1337_time_t* time);
ec_gt ds1337_get_time(ds1337_dev_t* dev, ds1337_time_t* time_ret);

ec_gt ds1337_set_alarm1(ds1337_dev_t* dev, const ds1337_time_t* alarm_time, ds1337_alarm1_rate_et rate);
ec_gt ds1337_set_alarm2(ds1337_dev_t* dev, const ds1337_time_t* alarm_time, ds1337_alarm2_rate_et rate);

ec_gt ds1337_enable_alarms(ds1337_dev_t* dev, bool enable_a1, bool enable_a2);

ec_gt ds1337_get_status(ds1337_dev_t* dev, uint8_t* status_ret);
ec_gt ds1337_clear_status(ds1337_dev_t* dev, uint8_t status_mask_to_clear);


/* example
ds1337_dev_t my_rtc;
ds1337_init_t rtc_cfg = {
    .int_mode = DS1337_INTCN_ALARM,   // 将外部中断引脚配置为响应闹钟
    .enable_oscillator = true         // 启动晶振开始计时
};

// 1. 初始化
ds1337_init(&my_rtc, I2CA_BASE, &rtc_cfg);

// 2. 设置当前时间 (比如: 2026年3月11日 04:56:00, 星期三)
ds1337_time_t current_time = {
    .year = 26, .month = 3, .date = 11, .day = 3, 
    .hours = 4, .minutes = 56, .seconds = 0
};
ds1337_set_time(&my_rtc, &current_time);

// 3. 设置闹钟1：每天的 08:30:00 准时触发
ds1337_time_t alarm_time = { .hours = 8, .minutes = 30, .seconds = 0 };
ds1337_set_alarm1(&my_rtc, &alarm_time, DS1337_A1_MATCH_HR_MIN_SEC);

// 4. 开启闹钟中断输出
ds1337_enable_alarms(&my_rtc, true, false);

// 5. 在主循环或中断中检测：
uint8_t status = 0;
ds1337_get_status(&my_rtc, &status);
if (status & DS1337_STATUS_A1F) {
    // 闹钟触发了！执行相关操作...
    
    // 别忘了清除标志位，否则 INT 引脚会一直保持低电平
    ds1337_clear_status(&my_rtc, DS1337_STATUS_A1F); 
}

*/

#ifdef __cplusplus
}
#endif

#endif /* DS1337_H */
