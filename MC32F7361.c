/******************************************************************************
;  *       @型号                 : MC32F7361
;  *       @创建日期             : 2021.12.21
;  *       @公司/作者            : SINOMCU-FAE
;  *       @晟矽微技术支持       : 2048615934
;  *       @晟矽微官网           : http://www.sinomcu.com/
;  *       @版权                 : 2021 SINOMCU公司版权所有.
;  *----------------------摘要描述---------------------------------
;
******************************************************************************/

#include "user.h"

// 毫秒级延时 (误差：在1%以内，1ms、10ms、100ms延时的误差均小于1%)
// 前提条件：FCPU = FHOSC / 4
void delay_ms(u32 xms)
{
    while (xms)
    {
        u32 i = 306;
        while (i--)
        {
            Nop();
        }
        xms--;
    }
}

#if USE_MY_DEBUG
// 通过一个引脚输出数据
// 发送一次数据约420ms
#define DEBUG_PIN P12D
void send_data_msb(u32 send_data)
{
    // 先发送格式头
    // __set_input_pull_up(); // 高电平
    DEBUG_PIN = 1;
    delay_ms(15);
    // __set_output_open_drain(); // 低电平
    DEBUG_PIN = 0;
    delay_ms(7); //

    for (u8 i = 0; i < 32; i++)
    {
        if ((send_data >> (32 - 1 - i)) & 0x01)
        {
            // 如果要发送逻辑1
            // __set_input_pull_up();  	   // 高电平
            DEBUG_PIN = 1;
            delay_ms(5); //
            // __set_output_open_drain(); // 低电平
            DEBUG_PIN = 0;
            delay_ms(10); //
        }
        else
        {
            // 如果要发送逻辑0
            // __set_input_pull_up();  	   // 高电平
            DEBUG_PIN = 1;
            delay_ms(5); //
            // __set_output_open_drain(); // 低电平
            DEBUG_PIN = 0;
            delay_ms(5); //
        }
    }

    // 最后，设置为低电平
    // __set_output_open_drain(); // 低电平
    DEBUG_PIN = 0;
    delay_ms(1);
    DEBUG_PIN = 1;
    delay_ms(1);
    DEBUG_PIN = 0;
}
#endif // #if USE_MY_DEBUG

/************************************************
;  *    @函数名          : CLR_RAM
;  *    @说明            : 清RAM
;  *    @输入参数        :
;  *    @返回参数        :
;  ***********************************************/
void CLR_RAM(void)
{
    for (FSR0 = 0; FSR0 < 0xff; FSR0++)
    {
        INDF0 = 0x00;
    }
    FSR0 = 0xFF;
    INDF0 = 0x00;
}
/************************************************
;  *    @函数名            : IO_Init
;  *    @说明              : IO初始化
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
void IO_Init(void)
{
    IOP0 = 0x00;   // io口数据位
    OEP0 = 0x3F;   // io口方向 1:out  0:in
    PUP0 = 0x00;   // io口上拉电阻   1:enable  0:disable
    PDP0 = 0x00;   // io口下拉电阻   1:enable  0:disable
    P0ADCR = 0x00; // io类型选择  1:模拟输入  0:通用io

    IOP1 = 0x00;   // io口数据位
    OEP1 = 0xFF;   // io口方向 1:out  0:in
    PUP1 = 0x00;   // io口上拉电阻   1:enable  0:disable
    PDP1 = 0x00;   // io口下拉电阻   1:enable  0:disable
    P1ADCR = 0x00; // io类型选择  1:模拟输入  0:通用io

    IOP2 = 0x00; // io口数据位
    OEP2 = 0x0F; // io口方向 1:out  0:in
    PUP2 = 0x00; // io口上拉电阻   1:enable  0:disable
    PDP2 = 0x00; // io口下拉电阻   1:enable  0:disablea

    PMOD = 0x00;  // P00、P01、P13 io端口值从寄存器读，推挽输出
    DRVCR = 0x80; // 普通驱动
    // DRVCR = 0x00; // 普通端口选择增强驱动电流输出
    // DRVCR = 0x30; //
}

/************************************************
;  *    @函数名            : ADC_Zero_ADJ
;  *    @说明              : ADC零点校准
;  *    @输入参数          :
;  *    @返回参数          :0 校准成功  1校准失败
;  *    用户按照需求选择是否校准
;  ***********************************************/
// uint8_t ADC_Zero_ADJ(void)
// {
//     ADCR0 = 0xeb;            // 使能ADC、12位数据H8L4、GND通道
//     ADCR1 = 0x80;            // 32分频、内部2V
//     ADCR2 = 0xff;            // 固定为15ADCLK
//     OSADJCR |= DEF_SET_BIT7; // 使能零点校准、负向修调
//     ADEOC = 0;
//     while (!ADEOC)
//         ;
//     if ((ADRH == 0) && ((ADRL & 0x0F) == 0)) // 结果是否为0
//     {
//         OSADJTD = 1; // 正向修调
//         OSADJCR |= 0x3F;
//         ADEOC = 0;
//         while (!ADEOC)
//             ;
//         if ((ADRH == 0) && ((ADRL & 0x0F) == 0)) // 结果是否为0
//         {
//             ADEN = 0;
//             return PASS;
//         }
//         else
//         {
//             while (1)
//             {
//                 if (OSADJCR & 0x3f)
//                 {
//                     OSADJCR--;
//                     ADEOC = 0;
//                     while (!ADEOC)
//                         ;
//                     if ((ADRH == 0) && ((ADRL & 0x0F) == 0)) // 结果是否为0
//                     {
//                         ADEN = 0;
//                         return PASS;
//                     }
//                 }
//                 else
//                 {
//                     ADEN = 0;
//                     return FAIL;
//                 }
//             }
//         }
//     }
//     else
//     {
//         while (1)
//         {
//             if ((OSADJCR & 0x3f) == 0x3f)
//             {
//                 ADEN = 0;
//                 return FAIL;
//             }
//             else
//             {
//                 OSADJCR++;
//                 ADEOC = 0;
//                 while (!ADEOC)
//                     ;
//                 if ((ADRH == 0) && ((ADRL & 0x0F) == 0)) // 结果是否为0
//                 {
//                     ADEN = 0;
//                     return PASS;
//                 }
//             }
//         }
//     }
// }
/************************************************
;  *    @函数名            : ADC_Init
;  *    @说明              : ADC初始化
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
void ADC_Init(void)
{
    OEP1 &= DEF_CLR_BIT0;   // P10为输入口
    P1ADCR |= DEF_SET_BIT0; // 使能P10模拟功能
    ADCR0 = 0xfb;           // 使能ADC、12位数据H8L4
    ADCR1 = 0x80;           // 32分频、内部2V
    ADCR2 = 0xff;           // 固定为15ADCLK
}

void adc_config(void)
{
    // 将对应的adc引脚配置为输入模式
    // 使能adc引脚的模拟功能
    // 配置adc模块

    // 注意：转换时钟越慢、采样时间越长，则越能过滤外部输入的波动，越能保证 AD 转换的精度

    P00PU = P00PD = 0; // 关闭上下拉
    P00OE = 0;         // 输入模式
    // P00MC = 1;         // 输入通道连通(可以不用写这一项)
    P00DC = 1; // 使能模拟功能

    ADCR0 = 0x0A; // 选择通道AN0、12位精度、不开始ad转换、使能adc
    // adc时钟使用FHIRC的256分频(最能过滤外部输入的波动，保证 AD 转换的精度)，
    // 参考电压选择内部2V
    ADCR1 = 0xE0;
    ADCR2 = 0xFF; // adc采样时间，固定为15ADCLK，只能写0xFF

    ADEN = 1;    // 开启ADC
    delay_ms(1); // 等待adc模块稳定
}

u16 adc_get_val(void)
{
    __adc_val_tmp = 0;
    __adc_val_sum = 0;
    __g_adcmax = 0;
    __g_adcmin = 0xFFFF;
    i = 0;

    for (i = 0; i < 20; i++)
    {
        ADEOC = 0; // 写0开始ad转换
        while (!ADEOC)
            ; // 等待转换完成
        __adc_val_tmp = ADRH;
        __adc_val_tmp = __adc_val_tmp << 4 | (ADRL & 0x0F);
        if (i < 2)
            continue; // 丢弃前两次采样的
        if (__adc_val_tmp > __g_adcmax)
            __g_adcmax = __adc_val_tmp; // 更新当前采集到的最大值
        if (__adc_val_tmp < __g_adcmin)
            __g_adcmin = __adc_val_tmp; // 更新当前采集到的最小值
        __adc_val_sum += __adc_val_tmp;
    }

    __adc_val_sum -= __g_adcmax;          // 去掉一个最大
    __adc_val_sum -= __g_adcmin;          // 去掉一个最小
    __adc_val_tmp = (__adc_val_sum >> 4); // 除以16，取平均值

    return __adc_val_tmp;
}

// 定时器0
void timer0_config(void)
{
    // 使能定时器，时钟源选择FCPU，时钟 32 分频
    // (FCPU = 8MHz,目前定时器时钟为0.25MHz，每4us计数一次)
    T0CR = 0x85;
    // T0CNT = 250 - 1; // 可以不写，定时器使能时会自动将T0LOAD的值载入T0CNT中
    T0LOAD = 250 - 1; // 计数250次，约1ms产生一次中断
    T0IE = 1;
}

/************************************************
;  *    @函数名            : Sys_Init
;  *    @说明              : 系统初始化
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
void Sys_Init(void)
{
    GIE = 0;
    CLR_RAM();
    IO_Init();

    // while (ADC_Zero_ADJ())
    //     ; // demo演示,校准失败一直校准，用户按照需求选择是否校准
    adc_config();

    timer0_config();

    GIE = 1;
}

/**
 * @brief 更新显存中的数据
 *
 * @param led_show_range 点亮第0个LED ~ 点亮第0~9个LED，范围：0~9
 */
void led_show_buf_update(const u8 led_show_range)
{
    // 先清空显存中的数据
    for (i = 0; i < ARRAY_SIZE(led_show_buff); i++)
    {
        led_show_buff[i] = LED_OFF;
    }

    // 给显存的第0个元素到 led_show_num 个元素，置一
    for (i = 0; i <= led_show_range; i++)
    {
        if (led_show_range < ARRAY_SIZE(led_show_buff))
        {
            led_show_buff[i] = LED_ON;
        }
    }
}

// 更新显示
void led_show_refresh(void)
{
    LED_0_PIN = led_show_buff[0];
    LED_1_PIN = led_show_buff[1];
    LED_2_PIN = led_show_buff[2];
    LED_3_PIN = led_show_buff[3];
    LED_4_PIN = led_show_buff[4];
    LED_5_PIN = led_show_buff[5];
    LED_6_PIN = led_show_buff[6];
    LED_7_PIN = led_show_buff[7];
    LED_8_PIN = led_show_buff[8];
    LED_9_PIN = led_show_buff[9];
}

void main(void)
{
    Sys_Init();

    // 刚上电，让LED全亮
    led_show_buf_update(ARRAY_SIZE(led_show_buff) - 1);
    led_show_refresh();
    delay_ms(1000);

    // 只保留第0格油量对应的LED点亮，其余LED全部熄灭
    led_show_buf_update(0);
    led_show_refresh();
    delay_ms(500);

    while (1)
    {

#if 0
        adc_val += adc_get_val(); // 调用一次约4.5ms
        if (adc_filter_cnt < 65535)
        {
            // 累加时，不能超过变量类型对应的最大值
            adc_filter_cnt++;
        }

        // 用定时器，连续检测 ADC_SCAN_TIME_MS 时间的 ad值,取平均值
        if (timer0_cnt >= ADC_SCAN_TIME_MS)
        {
            timer0_cnt = 0; // 清空计数值

            adc_val /= adc_filter_cnt;
            adc_filter_cnt = 0;
            // send_data_msb(adc_val); // 测试用

            adc_cmp_val_left = 0;
            adc_cmp_val_right = ADC_SCAN_VAL_MAX / 9 + ADC_DELTA_VAL;
            if (adc_val >= adc_cmp_val_left &&
                adc_val <= adc_cmp_val_right)
            {
                // 如果油量已经接近满格：
                cur_fuel_level = 9;
            }
            else
            {
                // 如果不是满格油量
                // 判断当前油量是否在第0级~第8级(第0格~第8格对应的位置)
                for (i = 1; i < 9; i++)
                {
                    adc_cmp_val_left = ADC_SCAN_VAL_MAX * i / 9 + ADC_DELTA_VAL;
                    adc_cmp_val_right = ADC_SCAN_VAL_MAX * (i + 1) / 9 - ADC_DELTA_VAL;

                    if (adc_val >= adc_cmp_val_left &&
                        adc_val <= adc_cmp_val_right)
                    {
                        cur_fuel_level = 9 - i; // 记录当前油量对应的级别(剩余 第 9 - i 格油量)
                        break;
                    }

                    // 判断当前油量是否在低0级(第0格对应的位置)
                    // 如果只有最后一格油量，熄灭LED1~9
                    if (adc_val >= ADC_SCAN_VAL_MAX - ADC_DELTA_VAL)
                    {
                        cur_fuel_level = 0;
                        break;
                    }
                }
            }

            // 判断完成后，清除adc值
            adc_val = 0;

        } // if (timer0_cnt >= ADC_SCAN_TIME_MS)

        // 更新LED显示（注意变化要连贯）
        if (last_fuel_level != cur_fuel_level)
        {
            if (last_fuel_level < cur_fuel_level)
            {
                // 如果之前检测到的油量等级(剩余格数)小于现在检测到的
                // LED显示的数量要逐渐增多，从左往右
                if (led_adjust_time_cnt >= LED_REFRESH_INTERVAL_MS)
                {
                    led_adjust_time_cnt = 0;
                    last_fuel_level++;
                }
            }
            else
            {
                // 如果之前检测到的油量等级(剩余格数)大于现在检测到的
                // LED显示的数量要逐渐减少，从右往左
                if (led_adjust_time_cnt >= LED_REFRESH_INTERVAL_MS)
                {
                    led_adjust_time_cnt = 0;
                    last_fuel_level--;
                }
            }

            led_adjust_time_cnt += ONE_CYCLE_TIME_MS;
            led_show_buf_update(last_fuel_level);
            led_show_refresh();
            // delay_ms(1);
        }
        else
        {
            // 如果之前检测到的油量等级(剩余格数)和现在的是相等的
            // 不用更新显示
            led_adjust_time_cnt = 0;
        }
#endif

        // LED_0_PIN = LED_ON;
        // LED_1_PIN = LED_ON;
        // LED_2_PIN = LED_ON;
        // LED_3_PIN = LED_ON;
        // LED_4_PIN = LED_ON;
        // LED_5_PIN = LED_ON;
        // LED_6_PIN = LED_ON;
        // LED_7_PIN = LED_ON;
        // LED_8_PIN = LED_ON;
        // LED_9_PIN = LED_ON;

        // 输入下拉，测试发现无法点亮LED
        // P16PD = 1;
        // P16OE = 0;

        // send_data_msb(adc_get_val());
        // delay_ms(100);

        if (adc_val < 4294967296 - 65535)
        {
            if (adc_filter_cnt < 65535)
            {
                adc_val += adc_get_val();
                adc_filter_cnt++;
            }
        }

        if (timer0_cnt >= ADC_SCAN_TIME_MS)
        {
            timer0_cnt = 0;

            adc_val /= adc_filter_cnt;

            for (i = 0; i < ARRAY_SIZE(adc_val_cmp_table); i++)
            {
                if (adc_val <= adc_val_cmp_table[i] + ADC_DELTA_VAL)
                {
                    cur_fuel_level = 9 - i;
                    break;
                }
            }

            adc_val = 0;
            adc_filter_cnt = 0;
        }

        // 更新LED显示（注意变化要连贯）
        if (last_fuel_level != cur_fuel_level)
        {
            flag_is_refresh_led = 1;

            if (led_adjust_time_cnt >= LED_REFRESH_INTERVAL_MS)
            {
                led_adjust_time_cnt = 0;
                if (last_fuel_level < cur_fuel_level)
                {
                    // 如果之前检测到的油量等级(剩余格数)小于现在检测到的
                    // LED显示的数量要逐渐增多，从左往右
                    last_fuel_level++;
                }
                else
                {
                    // 如果之前检测到的油量等级(剩余格数)大于现在检测到的
                    // LED显示的数量要逐渐减少，从右往左
                    last_fuel_level--;
                }

                led_show_buf_update(last_fuel_level);
                led_show_refresh();
                flag_is_refresh_led = 0;
            }
        }
        else
        {
            // 如果之前检测到的油量等级(剩余格数)和现在的是相等的
            // 不用更新显示
        }
    } // while (1)
}

/************************************************
;  *    @函数名            : interrupt
;  *    @说明              : 中断函数
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
void int_isr(void) __interrupt
{
    __asm;
    movra _abuf;
    swapar _PFLAG;
    movra _statusbuf;
    __endasm;

    if (T0IF & T0IE)
    {
        // P22D = ~P22D; // 测试定时器配置是否正确
        if (timer0_cnt < 65535) // 防止计数溢出
        {
            timer0_cnt++;
        }

        if (flag_is_refresh_led)
        {
            if (led_adjust_time_cnt < 65535)
            {
                led_adjust_time_cnt++;
            }
        }

        T0IF = 0; // 清除中断标志
    }

    __asm;
    swapar _statusbuf;
    movra _PFLAG;
    swapr _abuf;
    swapar _abuf;
    __endasm;
}

#if 0 // 测试500ms内连续检测ad再取平均值--测试通过

        adc_val += adc_get_val(); // 调用一次约4.5ms
        if (adc_filter_cnt < 65535)
        {
            // 累加时，不能超过变量类型对应的最大值
            adc_filter_cnt++;
        }

        if (timer0_cnt >= 5000)
        {
            timer0_cnt = 0;

            adc_val /= adc_filter_cnt;
            adc_filter_cnt = 0;
            send_data_msb(adc_val);
            adc_val = 0;
        }

#endif //

#if 0

        // LED显示测试：
        if (last_fuel_level == 9)
        {
            cur_fuel_level = 0;
        }
        else if (last_fuel_level == 0)
        {
            cur_fuel_level = 9;
        }

        // 如果之前检测到的油量等级(剩余格数)和现在的是相等的
        // 不用更新显示
        if (last_fuel_level != cur_fuel_level)
        {
            if (last_fuel_level < cur_fuel_level)
            {
                // 如果之前检测到的油量等级(剩余格数)小于现在检测到的
                // LED显示的数量要逐渐增多，从左往右

                if (led_adjust_time_cnt >= LED_REFRESH_INTERVAL_MS)
                {
                    led_adjust_time_cnt = 0;
                    last_fuel_level++;
                }
            }
            else
            {
                // 如果之前检测到的油量等级(剩余格数)大于现在检测到的
                // LED显示的数量要逐渐减少，从右往左
                if (led_adjust_time_cnt >= LED_REFRESH_INTERVAL_MS)
                {
                    led_adjust_time_cnt = 0;
                    last_fuel_level--;
                }
            }

            led_adjust_time_cnt += ONE_CYCLE_TIME_MS;
            led_show_buf_update(last_fuel_level);
            led_show_refresh();
            // delay_ms(1);
        }
        else
        {
            led_adjust_time_cnt = 0;
        }
#endif

/**************************** end of file *********************************************/
