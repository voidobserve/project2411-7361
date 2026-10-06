/******************************************************************************
;  *       @型号                   : MC32F7361
;  *       @创建日期               : 2021.12.21
;  *       @公司/作者              : SINOMCU-FAE
;  *       @晟矽微技术支持         : 2048615934
;  *       @晟矽微官网             : http://www.sinomcu.com/
;  *       @版权                   : 2021 SINOMCU公司版权所有.
;  *---------------------- 建议 ---------------------------------
;  *                   变量定义时使用全局变量
******************************************************************************/
#ifndef USER
#define USER
#include "mc32-common.h"
#include "MC32F7361.h"

/*****************************************************************
;       Function : Define variables
;*****************************************************************/
#define u8       unsigned char
#define u16      unsigned int
#define u32      unsigned long int
#define uint8_t  unsigned char
#define uint16_t unsigned int
#define uint32_t unsigned long int

#if 1
#define DEF_SET_BIT0 0x01
#define DEF_SET_BIT1 0x02
#define DEF_SET_BIT2 0x04
#define DEF_SET_BIT3 0x08
#define DEF_SET_BIT4 0x10
#define DEF_SET_BIT5 0x20
#define DEF_SET_BIT6 0x40
#define DEF_SET_BIT7 0x80

#define DEF_CLR_BIT0 0xFE
#define DEF_CLR_BIT1 0xFD
#define DEF_CLR_BIT2 0xFB
#define DEF_CLR_BIT3 0xF7
#define DEF_CLR_BIT4 0xEF
#define DEF_CLR_BIT5 0xDF
#define DEF_CLR_BIT6 0xBF
#define DEF_CLR_BIT7 0x7F

#define FAIL 1
#define PASS 0
#endif

#define USER_DEBUG_ENABLE 0

#define LED_0_PIN P17D // 最后一格油量对应的LED
#define LED_1_PIN P01D
#define LED_2_PIN P02D
#define LED_3_PIN P03D
#define LED_4_PIN P04D
#define LED_5_PIN P11D
#define LED_6_PIN P15D
#define LED_7_PIN P14D
#define LED_8_PIN P13D
#define LED_9_PIN P16D // 满格油量对应的LED

#define LED_ON  0 // LED点亮时，引脚对应的电平
#define LED_OFF 1 // LED熄灭时，引脚对应的电平

#define ADC_SCAN_VAL_MAX   (2211)                      // adc能检测到的最大的值
#define ADC_DEAD_ZONE_VAL  (ADC_SCAN_VAL_MAX / 10 / 2) // adc死区值
#define ADC_VAL_FULL_SCALE (4095)                      // adc满量程值
// 未接油量判定阈值：油量检测脚悬空时，ad值非常接近满量程
// #define ADC_VAL_UNCONNECTED_TH (ADC_VAL_FULL_SCALE - 100)
#define ADC_VAL_UNCONNECTED_TH (2493) // 使用跟样机接近的值

// LED刷新显示的时间间隔
#define LED_REFRESH_INTERVAL_MS                   (1000)
#define LED_REFRESH_INTERVAL_MS_IN_LOW_FUEL_ALERT (500)

#define ARRAY_SIZE(array) (sizeof(array) / sizeof(array[0]))

#define ADC_GET_VAL_INTERVAL 10 // adc获取值的间隔，单位：ms
// 获取adc平均值的时间间隔，单位：ms
#define ADC_GET_FILTER_VAL_INTERVAL (500)

// 低油量报警时间（单位:ms）：进入前需要连续检测到该时间，退出只需要 1s
// 一个检测周期 = adc均值更新周期(ADC_GET_FILTER_VAL_INTERVAL)，进入时间可在此配置
#define LOW_FUEL_ALERT_ENTER_TIME_MS (7500)
#define LOW_FUEL_ALERT_EXIT_TIME_MS  (1000)
// 注意：下面两个计数用的是 u8，配置后的值不能大于 255
#define LOW_FUEL_ALERT_ENTER_CNT                                               \
    (LOW_FUEL_ALERT_ENTER_TIME_MS / ADC_GET_FILTER_VAL_INTERVAL)
#define LOW_FUEL_ALERT_EXIT_CNT                                                \
    (LOW_FUEL_ALERT_EXIT_TIME_MS / ADC_GET_FILTER_VAL_INTERVAL)

//===============Global Function===============
void Sys_Init(void);
void CLR_RAM(void);
void IO_Init(void);

//============Define  Flag=================
typedef union
{
    unsigned char byte;
    struct
    {
        u8 bit0 : 1;
        u8 bit1 : 1;
        u8 bit2 : 1;
        u8 bit3 : 1;
        u8 bit4 : 1;
        u8 bit5 : 1;
        u8 bit6 : 1;
        u8 bit7 : 1;
    } bits;
} bit_flag;

#define flag_is_in_low_fuel_alert flag1.bits.bit0 // 是否在低油量提示中
// 是否检测到低油量提示对应的挡位(仅 ad 值接近未接阈值时置位)，低油量报警的候选条件
#define flag_is_low_fuel_detected flag1.bits.bit1

#endif

/**************************** end of file *********************************************/
