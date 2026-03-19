/*********************************************************************************************************************
* 文件名称          dshot600.c
* 功能说明          DShot600协议驱动实现 - 双核版本（仅通讯功能）
********************************************************************************************************************/

#include "dshot600.h"
#include "zf_common_headfile.h"

/*
 * DShot 驱动模块:
 * 负责电调通信初始化、油门帧发送和解锁状态维护。
 */

// 电机引脚数组
static gpio_pin_enum motor_pins[DSHOT_MOTOR_NUM] = {
    MOTOR1_PIN,
    MOTOR2_PIN,
    MOTOR3_PIN,
    MOTOR4_PIN
};

// 电机状态（保留，但不再使用内部状态管理）
motor_state_t motor_state = {
    .throttle = {0, 0, 0, 0},
    .armed = 0,
    .mode = MODE_ALL_SAME,
    .speed_level = 1
};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算DShot数据包的CRC校验
//-------------------------------------------------------------------------------------------------------------------
static uint8 dshot_calculate_crc(uint16 packet)
{
    uint8 crc = 0;
    uint16 data = packet;

    for(uint8 i = 0; i < 3; i++)
    {
        crc ^= (data & 0x0F);
        data >>= 4;
    }

    return crc;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送一个DShot位
//-------------------------------------------------------------------------------------------------------------------
static inline void dshot_send_bit(gpio_pin_enum pin, uint8 bit)
{
    volatile uint16 i;

    if(bit)
    {
        // 发送逻辑1: 高电平1.25us, 低电平0.417us
        gpio_set_level(pin, 1);
        for(i = 0; i < 60; i++)
        {
            __nop();
            __nop();
            __nop();
            __nop();
        }

        gpio_set_level(pin, 0);
        for(i = 0; i < 20; i++)
        {
            __nop();
            __nop();
            __nop();
            __nop();
        }
    }
    else
    {
        // 发送逻辑0: 高电平0.625us, 低电平1.042us
        gpio_set_level(pin, 1);
        for(i = 0; i < 30; i++)
        {
            __nop();
            __nop();
            __nop();
            __nop();
        }

        gpio_set_level(pin, 0);
        for(i = 0; i < 50; i++)
        {
            __nop();
            __nop();
            __nop();
            __nop();
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送DShot数据包
//-------------------------------------------------------------------------------------------------------------------
static void dshot_send_packet(gpio_pin_enum pin, uint16 packet)
{
    // 关闭中断，确保时序准确
    boolean cpu_sr = IfxCpu_disableInterrupts();

    // 发送16位数据，从MSB开始
    for(int8 i = 15; i >= 0; i--)
    {
        dshot_send_bit(pin, (packet >> i) & 0x01);
    }

    // 恢复中断
    IfxCpu_restoreInterrupts(cpu_sr);
}

/*
 * DShot 初始化入口:
 * 配置四路电机输出引脚、LED 指示灯，并发送若干帧停机命令让电调进入待命状态。
 */
void dshot_init(void)
{
    // 初始化四个电机引脚为推挽输出，初始低电平
    for(uint8 i = 0; i < DSHOT_MOTOR_NUM; i++)
    {
        gpio_init(motor_pins[i], GPO, 0, GPO_PUSH_PULL);
    }

    // 初始化LED（可选，如果CPU1不需要可以注释掉）
    gpio_init(LED1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(LED2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(LED3, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(LED4, GPO, GPIO_LOW, GPO_PUSH_PULL);

    // 延时等待电调初始化
    system_delay_ms(100);

    // 发送几次0油门信号，确保电调进入正常模式
    for(uint8 i = 0; i < 10; i++)
    {
        dshot_motor_stop_all();
        system_delay_ms(10);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送油门值到指定电机
//-------------------------------------------------------------------------------------------------------------------
void dshot_send_throttle(uint8 motor_index, uint16 throttle, uint8 telemetry)
{
    if(motor_index >= DSHOT_MOTOR_NUM)
        return;

    // 限制油门值范围
    if(throttle > DSHOT_MAX_THROTTLE)
        throttle = DSHOT_MAX_THROTTLE;

    // 构建DShot数据包
    uint16 packet = (throttle << 1) | (telemetry & 0x01);
    uint8 crc = dshot_calculate_crc(packet);
    packet = (packet << 4) | crc;

    // 发送数据包
    dshot_send_packet(motor_pins[motor_index], packet);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     同时设置四个电机的油门值
//-------------------------------------------------------------------------------------------------------------------
void dshot_set_all_throttle(uint16 throttle1, uint16 throttle2, uint16 throttle3, uint16 throttle4)
{
    uint16 throttles[4] = {throttle1, throttle2, throttle3, throttle4};

    for(uint8 i = 0; i < DSHOT_MOTOR_NUM; i++)
    {
        dshot_send_throttle(i, throttles[i], 0);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送特殊命令到指定电机
//-------------------------------------------------------------------------------------------------------------------
void dshot_send_command(uint8 motor_index, uint8 command)
{
    if(motor_index >= DSHOT_MOTOR_NUM)
        return;

    // 特殊命令需要连续发送多次
    for(uint8 i = 0; i < 10; i++)
    {
        dshot_send_throttle(motor_index, command, 0);
        system_delay_ms(1);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止所有电机
//-------------------------------------------------------------------------------------------------------------------
void dshot_motor_stop_all(void)
{
    for(uint8 i = 0; i < DSHOT_MOTOR_NUM; i++)
    {
        motor_state.throttle[i] = 0;
        dshot_send_throttle(i, DSHOT_DISARM, 0);
    }
    motor_state.armed = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按键扫描处理（已禁用，由CPU1统一处理）
//-------------------------------------------------------------------------------------------------------------------
void dshot_key_scan(void)
{
    // 此函数已废弃，按键由CPU1的key_scan_cpu1()处理
    // 保留函数体防止编译错误，但不执行任何操作
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新电机输出（已禁用，由CPU0/CPU1通过共享变量控制）
//-------------------------------------------------------------------------------------------------------------------
void dshot_update_motors(void)
{
    // 此函数已废弃，电机油门由共享变量shared_motor_throttle控制
    // DShot信号发送在CPU1的中断中完成
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取电机状态
//-------------------------------------------------------------------------------------------------------------------
motor_state_t* dshot_get_state(void)
{
    return &motor_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置解锁状态
//-------------------------------------------------------------------------------------------------------------------
void dshot_set_armed(uint8 armed)
{
    motor_state.armed = armed;
    if(!armed)
    {
        dshot_motor_stop_all();
    }
    gpio_set_level(LED1, armed);
}
