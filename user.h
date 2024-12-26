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
#define u8 unsigned char
#define u16 unsigned int
#define u32 unsigned long int
#define uint8_t unsigned char
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

#define USE_MY_DEBUG 0

// 控制LED的引脚定义
// #define LED_0_PIN P20D // 最后一格油量对应的LED
// #define LED_1_PIN P01D
// #define LED_2_PIN P02D
// #define LED_3_PIN P03D
// #define LED_4_PIN P04D
// #define LED_5_PIN P11D
// #define LED_6_PIN P15D
// #define LED_7_PIN P14D
// #define LED_8_PIN P13D
// #define LED_9_PIN P05D // 满格油量对应的LED

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

#define LED_ON 0  // LED点亮时，引脚对应的电平
#define LED_OFF 1 // LED熄灭时，引脚对应的电平

#define ADC_SCAN_VAL_MAX (2250) // adc能检测到的最大的值(要注意不能大于变量的最大值)
// #define ADC_DELTA_VAL (34) // adc死区值(要注意不能大于变量的最大值)
#define ADC_DELTA_VAL (ADC_SCAN_VAL_MAX / 10 / 2) // adc死区值(要注意不能大于变量的最大值)
#define ADC_SCAN_TIME_MS (500)                    // adc扫描周期（单位:ms）(要注意不能大于变量的最大值)
#define LED_REFRESH_INTERVAL_MS (700)             // LED刷新显示的时间间隔(要注意不能大于变量的最大值)
// #define ONE_CYCLE_TIME_MS (4)                     // 主循环一次所需的时间(单位：ms)(要注意不能大于变量的最大值)

#define ARRAY_SIZE(array) (sizeof(array) / sizeof(array[0]))

u8 led_show_buff[10]; // led显存

/*
    假设检测到的电压范围：0~2250
    那么每级电压对应 225 单位的ad值
    225 / 2 == 112.5

    225
    450
    675
    900
    1125

    1350
    1575
    1800
    2025
    2250

    取中间值作为分隔,中间值的一半作为死区
    337.5
    562.5
    787.5
    1012.5
    1237.5

    1462.5
    1687.5
    1912.5
    2137.5
*/

// 定义油量的比较值
const u16 adc_val_cmp_table[] = { 
    ADC_SCAN_VAL_MAX / 10 * 1, // 10格油量，最大油量
    ADC_SCAN_VAL_MAX / 10 * 2, // 9格油量
    ADC_SCAN_VAL_MAX / 10 * 3, // 8格油量
    ADC_SCAN_VAL_MAX / 10 * 4, // 7格油量
    ADC_SCAN_VAL_MAX / 10 * 5, // 6格油量
    ADC_SCAN_VAL_MAX / 10 * 6, // 5格油量
    ADC_SCAN_VAL_MAX / 10 * 7, // 4格油量
    ADC_SCAN_VAL_MAX / 10 * 8, // 3格油量
    ADC_SCAN_VAL_MAX / 10 * 9, // 2格油量 
    ADC_SCAN_VAL_MAX / 10 * 10, // 1格油量 
};

//===============Global Variable===============
u8 i;              // 循环计数值

u8 last_fuel_level; // 存放之前检测到的油量
u8 cur_fuel_level;  // 存放当前检测到的油量(目前分为10级,0~9)

u16 adc_filter_cnt; // 一个检测周期中(两次更新LED显示的时间间隔),存放adc的采集次数,用于后续滤波
u32 adc_val; // 存放采集到的ad值

// u16 adc_cmp_val_left;  // 存放adc比较值--左值
// u16 adc_cmp_val_right; // 存放adc比较值--右值

volatile u16 timer0_cnt; // 定时器0计数值

volatile u16 led_adjust_time_cnt; // 调整LED显示时用到的时间计数

// ===========================================================
// adc_get_val()函数中使用到的变量
u16 __adc_val_tmp; // 临时存放单次转换中采集的ad值
u32 __adc_val_sum; // 存放多次采集到的ad值
u16 __g_adcmax;
u16 __g_adcmin;
// adc_get_val()函数中使用到的变量
// ===========================================================

//===============Global Function===============
void Sys_Init(void);
void CLR_RAM(void);
void IO_Init(void);

// uint8_t ADC_Zero_ADJ(void);

// 中断服务函数使用到的变量
u8 abuf;
u8 statusbuf;

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
volatile bit_flag flag1;

#define flag_is_refresh_led flag1.bits.bit0 // 是否要刷新led

#endif

/**************************** end of file *********************************************/
