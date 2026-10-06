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

// 滑动平均：16 点窗口(2 的幂)，用 u16 累加 + 右移求平均
// 注意：不要写 `tmp_val_u32 += adc_val_buf[i]`（u32 累加器 += volatile 数组元素）：
//   本工具链(WinScopeIDE v1.33.01 的 sdcc 后端)会漏掉"把样本高字节载入 A"的那句 MOVAR，
//   低字节相加没有进位时会把低字节加到高位上，和值被吹大约 17 倍
//   （实测：真实均值 939 → 算出来 0x3F38 = 16184）。
#define ADC_VAL_BUF_LEN 16
volatile u16 adc_val_buf[ADC_VAL_BUF_LEN] = {0};
volatile u8 adc_val_buf_cnt = 0;
volatile u8 is_adc_val_buf_initialized = 0;
// 16 个样本的和：最大 16 * 4095 = 65520 ≤ 65535，u16 不会溢出
volatile u16 adc_sum = 0;

volatile u16 adc_get_filter_val_cnt = 0;

volatile u16 tmp_val_u16 = 0;

volatile u8 led_show_buff[10]; // led显存

enum
{
    ADC_VAL_LEV_9 = ADC_SCAN_VAL_MAX * 1 / 10,  // 10格油量，最大油量
    ADC_VAL_LEV_8 = ADC_SCAN_VAL_MAX * 2 / 10,  // 9格油量
    ADC_VAL_LEV_7 = ADC_SCAN_VAL_MAX * 3 / 10,  // 8格油量
    ADC_VAL_LEV_6 = ADC_SCAN_VAL_MAX * 4 / 10,  // 7格油量
    ADC_VAL_LEV_5 = ADC_SCAN_VAL_MAX * 5 / 10,  // 6格油量
    ADC_VAL_LEV_4 = ADC_SCAN_VAL_MAX * 6 / 10,  // 5格油量
    ADC_VAL_LEV_3 = ADC_SCAN_VAL_MAX * 7 / 10,  // 4格油量
    ADC_VAL_LEV_2 = ADC_SCAN_VAL_MAX * 8 / 10,  // 3格油量
    ADC_VAL_LEV_1 = ADC_SCAN_VAL_MAX * 9 / 10,  // 2格油量
    ADC_VAL_LEV_0 = ADC_SCAN_VAL_MAX * 10 / 10, // 1格油量
};

// 定义油量的比较值
const u16 adc_val_cmp_table[] = {
    ADC_VAL_LEV_9, ADC_VAL_LEV_8, ADC_VAL_LEV_7, ADC_VAL_LEV_6, ADC_VAL_LEV_5,
    ADC_VAL_LEV_4, ADC_VAL_LEV_3, ADC_VAL_LEV_2, ADC_VAL_LEV_1, ADC_VAL_LEV_0,
};

//===============Global Variable===============
volatile u8 i; // 循环计数值

volatile u8 last_fuel_lev; // 存放之前检测到的油量等级
volatile u8 cur_fuel_lev;  // 存放当前检测到的油量等级

// 低油量报警的进入/退出计数，单位：ADC_GET_FILTER_VAL_INTERVAL(一次ad均值更新的周期)
volatile u8 low_fuel_alert_enter_cnt; // 连续检测到低油量的次数
volatile u8 low_fuel_alert_exit_cnt;  // 连续检测到油量正常的次数

volatile u16 adc_val; // 存放采集到的ad值

volatile u16 led_refresh_time_cnt; // 调整LED显示时用到的时间计数

// 中断服务函数使用到的变量
u8 abuf;
u8 statusbuf;

volatile bit_flag flag1;

// 毫秒级延时 (误差：在1%以内，1ms、10ms、100ms延时的误差均小于1%)
// 前提条件：FCPU = FHOSC / 4
void delay_ms(u32 xms)
{
    while (xms) {
        u32 i = 306;
        while (i--) {
            Nop();
        }
        xms--;
    }
}

#if USER_DEBUG_ENABLE
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

    for (u8 i = 0; i < 32; i++) {
        if ((send_data >> (32 - 1 - i)) & 0x01) {
            // 如果要发送逻辑1
            // __set_input_pull_up();  	   // 高电平
            DEBUG_PIN = 1;
            delay_ms(5); //
            // __set_output_open_drain(); // 低电平
            DEBUG_PIN = 0;
            delay_ms(10); //
        } else {
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

// ================= 软件串口(主循环翻转 IO，只发不收) =================
//  TX 脚 : P12D，空闲为高电平
//  帧格式: 8N1 —— 起始位(低) + 8 数据位(低位先发) + 停止位(高)
//  调用  : 在主循环里调用，函数内部阻塞发送(9600bps 发 4 字节约 4.2ms)
//  FCPU  : 8MHz
#define UART_TX_PIN P12D
#define UART_BAUD   9600UL
// #define UART_BAUD 2400UL

// 实测标定(逻辑分析仪 @FCPU=8MHz)：
//   UART_CAL_LOOPS 次忙等 + 该位的固定开销 = UART_CAL_US 微秒
//   实测 31 次 → 115.5us(约 3.7us/次，比 delay_ms() 注释里的 3.26us 慢约 11%)
// 所以直接按比例换算一位的循环次数：
//   9600bps → 104 * 31 / 115 ≈ 28 次 ≈ 104.3us(理论值 104.17us)
//   2400bps → 416 * 31 / 115 ≈ 112 次   19200bps → 52 * 31 / 115 ≈ 14 次
// 换波特率只改 UART_BAUD；要更准就用逻辑分析仪量位宽后再改标定值
#define UART_CAL_LOOPS 31UL  // 标定时的忙等次数
#define UART_CAL_US    115UL // 标定时实测的位宽(单位:us)
#define UART_BIT_LOOPS                                                         \
    (((1000000UL / UART_BAUD) * UART_CAL_LOOPS + UART_CAL_US / 2) / UART_CAL_US)
// 停止位补偿：从停止位写脚到下一字节起始位写脚，中间还有
// "循环退出+RETURN + uart_send_u32 取字节 + 下一字节的 CALL 和函数入口"，
// 实测多出约 7us(≈ 2 次忙等)，这里减掉，保证停止位也是 1 个位宽
#define UART_STOP_COMP_LOOPS 2UL

// 发送一个字节(8N1)
void uart_send_byte(u8 dat)
{
    u8 i;
    u32 n;

    GIE = 0; // 关全局中断

    // 起始位
    UART_TX_PIN = 0;
    n = UART_BIT_LOOPS;
    while (n--) {
        Nop();
    }

    // 8 个数据位，低位先发。
    // 注意：每个位走完全相同的代码，位宽才一致(不要在循环里加分支)
    for (i = 0; i < 8; i++) {
        UART_TX_PIN = (u8)(dat & 0x01);
        n = UART_BIT_LOOPS;
        while (n--) {
            Nop();
        }
        dat >>= 1; // 准备下一位(固定开销)
    }

    // 停止位：少等 UART_STOP_COMP_LOOPS 次，补掉"循环退出+RETURN+
    // 取字节+CALL+函数入口"的字节边界开销，使停止位也是 1 个位宽
    UART_TX_PIN = 1;
    n = UART_BIT_LOOPS - UART_STOP_COMP_LOOPS;
    while (n--) {
        Nop();
    }

    GIE = 1; // 开全局中断
}

// 发送 u32：拆成 4 字节，高字节先发(串口助手按 hex 直接读数值)
void uart_send_u32(u32 dat)
{
    uart_send_byte((u8)(dat >> 24));
    uart_send_byte((u8)(dat >> 16));
    uart_send_byte((u8)(dat >> 8));
    uart_send_byte((u8)dat);
}

#endif

/************************************************
;  *    @函数名          : CLR_RAM
;  *    @说明            : 清RAM
;  *    @输入参数        :
;  *    @返回参数        :
;  ***********************************************/
void CLR_RAM(void)
{
    for (FSR0 = 0; FSR0 < 0xff; FSR0++) {
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

// 开启单次转换，并获取转换结果
u16 adc_convert(void)
{
    u8 i = 0;
    u8 val;
    /*
        这里不能用 volatile u32来计算，可能会被编译器优化，导致计算错误
        改用 u16，哪怕采集到16次最大值4095，求和之后的值是65520，不会超过 65535
    */
    u16 ret = 0;
    for (i = 0; i < 16; i++) {
        ADEOC = 0; // 写0开始ad转换
        while (!ADEOC)
            ; // 等待转换完成
        val = ADRH;
        ret += (u16)val << 4 | (ADRL & 0x0F);
    }

    return (ret >> 4);
}

// 定时器0
void timer0_config(void)
{
    // 使能定时器，时钟源选择FCPU，时钟 32 分频
    // (FCPU = 8MHz,目前定时器时钟为0.25MHz，每4us计数一次)
    T0CR = 0x85;
    // T0CNT = 250 - 1; // 可以不写，定时器使能时会自动将T0LOAD的值载入T0CNT中
    T0LOAD = (u8)(250 - 1); // 计数250次，约 1ms 产生一次中断
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

    adc_config();

    timer0_config();

    GIE = 1;
}

/**
 * @brief 更新显存中的数据
 *
 * @param led_show_range 
 *      0： 所有led熄灭
 *      1： 点亮第一格led
 *      2： 点亮第一格和第二格led
 *      ...
 *      10： 点亮第一格到第十格led
 * 
 */
void led_show_buf_update(const u8 led_show_range)
{
    // 先清空显存中的数据
    for (i = 0; i < ARRAY_SIZE(led_show_buff); i++) {
        led_show_buff[i] = LED_OFF;
    }

    // 给显存的第0个元素到 led_show_num 个元素，置一
    // for (i = 0; i <= led_show_range; i++) {
    //     if (led_show_range < ARRAY_SIZE(led_show_buff)) {
    //         led_show_buff[i] = LED_ON;
    //     }
    // }

    for (i = 0; i < ARRAY_SIZE(led_show_buff); i++) {
        if (led_show_range >= 1 && (led_show_range - 1) >= i) {
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
    led_show_buf_update(ARRAY_SIZE(led_show_buff));
    led_show_refresh();
    delay_ms(1000);

    // 只保留第0格油量对应的LED点亮，其余LED全部熄灭
    led_show_buf_update(1);
    led_show_refresh();
    delay_ms(500);

    // 上电后，默认为1格油量
    last_fuel_lev = 1;
    cur_fuel_lev = 1;

    while (1) {
        tmp_val_u16 = adc_convert();
#if USER_DEBUG_ENABLE
        // 软件串口发送(波特率见 UART_BAUD)；想换回原来的自定义协议就改成
        uart_send_u32(tmp_val_u16);
        uart_send_u32(0x12345678);
#endif

        // 每个主循环采一个样本放进环形缓冲(窗口时长 = 16 × 主循环周期)
        if (0 == is_adc_val_buf_initialized) {
            // 上电第一次：整窗先填成同一个值，避免用 0 参与平均把均值拉低
            is_adc_val_buf_initialized = 1;
            for (i = 0; i < ARRAY_SIZE(adc_val_buf); i++) {
                adc_val_buf[i] = tmp_val_u16;
            }
            adc_val_buf_cnt = 0;
        } else {
            adc_val_buf[adc_val_buf_cnt] = tmp_val_u16;
            adc_val_buf_cnt++;
            if (adc_val_buf_cnt >= ARRAY_SIZE(adc_val_buf)) {
                adc_val_buf_cnt = 0;
            }
        }

        if (adc_get_filter_val_cnt >= ADC_GET_FILTER_VAL_INTERVAL) {
            // 获取滑动平均后的ad值的时间周期到来
            GIE = 0; // 关中断，保证16位计数清零不会被中断打断
            adc_get_filter_val_cnt = 0; // 原子清零
            GIE = 1;
            // 16 点滑动平均：u16 累加(最大 65520，不溢出)，除 16 用右移
            // (+8 是四舍五入，误差 ≤ ±0.5 LSB；每次重算整窗，误差不累积)
            adc_sum = 0;
            for (i = 0; i < ARRAY_SIZE(adc_val_buf); i++) {
                adc_sum += adc_val_buf[i];
            }
            adc_val = (u16)((adc_sum + 8) >> 4);

#if USER_DEBUG_ENABLE
            uart_send_u32(adc_val);
            uart_send_u32(0x22222222);
#endif

            cur_fuel_lev = 0; // 默认为低油量提示对应的挡位
            for (i = 0; i < ARRAY_SIZE(adc_val_cmp_table); i++) {
                if (adc_val <= adc_val_cmp_table[i]) {
                    cur_fuel_lev = ARRAY_SIZE(adc_val_cmp_table) - i;
                    break;
                }
            }

#if USER_DEBUG_ENABLE
            uart_send_u32(cur_fuel_lev);
            uart_send_u32(0x33333333);
#endif

            /*
                adc_val 的区间划分：
                  1) adc_val <= ADC_SCAN_VAL_MAX
                     ：正常油量，挡位 1~10
                  2) ADC_SCAN_VAL_MAX < adc_val < ADC_VAL_UNCONNECTED_TH
                     ：已接油量传感器，只是超出了挡位表的量程，按 1 格油量显示，不进入低油量提示
                  3) adc_val >= ADC_VAL_UNCONNECTED_TH
                     ：ad 值非常接近满量程，认为油量检测脚未接(悬空)，
                       按最低格(最后一格)显示，并作为低油量提示的候选
            */
            flag_is_low_fuel_detected = 0;
            if (cur_fuel_lev == 0) {
                // 超出挡位表量程，显示上按最低格处理，避免灭灯
                cur_fuel_lev = 1;
                if (adc_val >= ADC_VAL_UNCONNECTED_TH) {
                    // ad值接近满量程 → 油量检测脚未接(悬空)，作为低油量提示的候选
                    flag_is_low_fuel_detected = 1;
                }
            }

            // 低油量提示：连续检测到"油量检测脚未接" LOW_FUEL_ALERT_ENTER_TIME_MS 后才进入，
            // 连续检测到油量正常 LOW_FUEL_ALERT_EXIT_TIME_MS 后就退出
            if (flag_is_low_fuel_detected) {
                low_fuel_alert_exit_cnt = 0; // 重新检测到报警条件，退出计数清零
                // 等油量指示灯已经降到只剩最后 1 格之后，才开始对进入低油量提示计数
                if (!flag_is_in_low_fuel_alert && last_fuel_lev <= 1) {
                    low_fuel_alert_enter_cnt++;
                    if (low_fuel_alert_enter_cnt >= LOW_FUEL_ALERT_ENTER_CNT) {
                        flag_is_in_low_fuel_alert = 1;
                        // 切换显示方式时清空旧的计时，保证第一次闪烁也是完整的 500ms
                        GIE = 0;
                        led_refresh_time_cnt = 0;
                        GIE = 1;
                    }
                }
            } else {
                low_fuel_alert_enter_cnt = 0; // 检测到正常油量，进入计数清零
                if (flag_is_in_low_fuel_alert) {
                    // 如果在低油量提示中，检测到正常油量，开始退出计数
                    low_fuel_alert_exit_cnt++;
                    if (low_fuel_alert_exit_cnt >= LOW_FUEL_ALERT_EXIT_CNT) {
                        flag_is_in_low_fuel_alert = 0;
                        // 退出报警后从最低格开始，按 1 格/秒 缓慢递增，不会直接跳到对应格数
                        // 切换显示方式时清空旧的计时
                        GIE = 0;
                        led_refresh_time_cnt = 0;
                        GIE = 1;

                        last_fuel_lev = 1;
                        led_show_buf_update(last_fuel_lev);
                        led_show_refresh();
                    }
                }
            }
        }

        // 更新LED显示（注意变化要连贯）
        if (flag_is_in_low_fuel_alert) {
            if (led_refresh_time_cnt >=
                LED_REFRESH_INTERVAL_MS_IN_LOW_FUEL_ALERT) {
                GIE = 0; // 关中断，保证16位计数清零不会被中断打断
                led_refresh_time_cnt = 0;
                GIE = 1;

                // 判断最后一格指示灯有没有点亮
                if (led_show_buff[0] == LED_OFF) {
                    // 只点亮最后一格指示灯
                    led_show_buf_update(1);
                } else {
                    // 熄灭所有led
                    led_show_buf_update(0);
                }
                led_show_refresh();
            }
        } else {
            if (led_refresh_time_cnt >= LED_REFRESH_INTERVAL_MS) {
                GIE = 0; // 关中断，保证16位计数清零不会被中断打断
                led_refresh_time_cnt = 0;
                GIE = 1;
                if (last_fuel_lev < cur_fuel_lev) {
                    // 如果之前检测到的油量等级(剩余格数)小于现在检测到的
                    // LED显示的数量要逐渐增多，从左往右
                    last_fuel_lev++;
                } else if (last_fuel_lev > cur_fuel_lev) {
                    // 如果之前检测到的油量等级(剩余格数)大于现在检测到的
                    // LED显示的数量要逐渐减少，从右往左
                    last_fuel_lev--;
                }

                led_show_buf_update(last_fuel_lev);
                led_show_refresh();
            }
        }
    }
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

    if (T0IF & T0IE) {
        T0IF = 0; // 清除中断标志

        if (adc_get_filter_val_cnt < ((u16)-1)) {
            adc_get_filter_val_cnt++;
        }

        if (led_refresh_time_cnt < ((u16)-1)) {
            led_refresh_time_cnt++;
        }
    }

    __asm;
    swapar _statusbuf;
    movra _PFLAG;
    swapr _abuf;
    swapar _abuf;
    __endasm;
}

/**************************** end of file *********************************************/
