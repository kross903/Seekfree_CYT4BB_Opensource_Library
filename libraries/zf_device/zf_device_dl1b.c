/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一�?基于官方 SDK 接口的�??三方开源库
* Copyright (c) 2022 SEEKFREE 逐�?��?�技
*
* �?文件�? CYT4BB 开源库的一部分
*
* CYT4BB 开源库 �?免费�?�?
* 您可以根�?�?由软件基金会发布�? GPL（GNU General Public License，即 GNU通用�?共�?�可证）的条�?
* �? GPL 的�??3版（�? GPL3.0）或（您选择的）任何后来的版�?，重新发布和/或修改它
*
* �?开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更�?�细节�?�参�? GPL
*
* 您应该在收到�?开源库的同时收到一�? GPL 的副�?
* 如果没有，�?�参�?<https://www.gnu.org/licenses/>
*
* 额�?�注明：
* �?开源库使用 GPL3.0 开源�?�可证协�? 以上许可申明为译文版�?
* 许可申明英文版在 libraries/doc 文件夹下�? GPL3_permission_statement.txt 文件�?
* 许可证副�?�? libraries 文件夹下 即�?�文件夹下的 LICENSE 文件
* 欢迎各位使用并传�?�?程序 但修改内容时必须保留逐�?��?�技的版权声明（即本声明�?
*
* 文件名称          zf_device_dl1b
* �?司名�?          成都逐�?��?�技有限�?�?
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环�?          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* �?改�?�录
* 日期              作�?                备注
* 2024-01-12       pudding           first version
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义�?
*                   ------------------------------------
*                   模块管脚            单片机�?�脚
*                   SCL                 查看 zf_device_dl1b.h �? DL1B_SCL_PIN  宏定�?
*                   SDA                 查看 zf_device_dl1b.h �? DL1B_SDA_PIN  宏定�?
*                   XS                  查看 zf_device_dl1b.h �? DL1B_XS_PIN  宏定�?
*                   VCC                 5V 电源
*                   GND                 电源�?
*                   ------------------------------------
********************************************************************************************************************/

#include "zf_common_debug.h"

#include "zf_driver_delay.h"
#include "zf_driver_exti.h"
#include "zf_driver_soft_iic.h"

#include "zf_device_dl1b.h"
#include "zf_device_config.h"
#include "zf_device_type.h"

uint8 dl1b_init_flag = 0;
uint8 dl1b_finsh_flag = 0;
uint16 dl1b_distance_mm = 8192;

#if DL1B_USE_SOFT_IIC
static soft_iic_info_struct dl1b_iic_struct;

#define dl1b_transfer_8bit_array(tdata, tlen, rdata, rlen)      (soft_iic_transfer_8bit_array(&dl1b_iic_struct, (tdata), (tlen), (rdata), (rlen)))
#else
#error "暂不�?持硬件IIC通�??"
#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简�?     返回以�??米为单位的范围�?�数
// 参数说明     void
// 返回参数     void
// 使用示例     dl1b_get_distance();
// 备注信息     在开始单次射程测量后也调用�?�函�?
//-------------------------------------------------------------------------------------------------------------------
void dl1b_get_distance (void)
{
    if(dl1b_init_flag)
    {
        uint8 data_buffer[3] = {0};
        int16 dl1b_distance_temp = 0;

        data_buffer[0] = DL1B_GPIO__TIO_HV_STATUS >> 8;
        data_buffer[1] = DL1B_GPIO__TIO_HV_STATUS & 0xFF;
        dl1b_transfer_8bit_array(data_buffer, 2, &data_buffer[2], 1);

        if(data_buffer[2])
        {

            data_buffer[0] = DL1B_SYSTEM__INTERRUPT_CLEAR >> 8;
            data_buffer[1] = DL1B_SYSTEM__INTERRUPT_CLEAR & 0xFF;
            data_buffer[2] = 0x01;
            dl1b_transfer_8bit_array(data_buffer, 3, data_buffer, 0);// clear Interrupt

            data_buffer[0] = DL1B_RESULT__RANGE_STATUS >> 8;
            data_buffer[1] = DL1B_RESULT__RANGE_STATUS & 0xFF;
            dl1b_transfer_8bit_array(data_buffer, 2, &data_buffer[2], 1);

            if(0x89 == data_buffer[2])
            {
                data_buffer[0] = DL1B_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 >> 8;
                data_buffer[1] = DL1B_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 & 0xFF;
                dl1b_transfer_8bit_array(data_buffer, 2, data_buffer, 2);
                dl1b_distance_temp = data_buffer[0];
                dl1b_distance_temp = (dl1b_distance_temp << 8) | data_buffer[1];

                if(dl1b_distance_temp > 4000 || dl1b_distance_temp < 0)
                {
                    dl1b_distance_mm = 8192;
                    dl1b_finsh_flag = 0;
                }
                else
                {
                    dl1b_distance_mm = dl1b_distance_temp;
                    dl1b_finsh_flag = 1;
                }
            }
            else
            {
                dl1b_distance_mm = 8192;
                dl1b_finsh_flag = 0;
            }
        }
        else
        {
            dl1b_distance_mm = 8192;
            dl1b_finsh_flag = 0;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简�?     DL1B INT �?�?响应处理函数
// 参数说明     void
// 返回参数     void
// 使用示例     dl1b_int_handler();
// 备注信息     �?函数需要在 DL1B_INT_PIN 对应的�?�部�?�?处理函数�?调用
//-------------------------------------------------------------------------------------------------------------------
void dl1b_int_handler (void)
{
#if DL1B_INT_ENABLE
    dl1b_get_distance();
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简�?     初�?�化 DL1B
// 参数说明     void
// 返回参数     uint8           1-初�?�化失败 0-初�?�化成功
// 使用示例     dl1b_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 dl1b_init (void)
{
    uint8   return_state    = 0;
    uint8   data_buffer[2 + sizeof(dl1b_config_file)];
    uint16  time_out_count  = 0;

#if DL1B_USE_SOFT_IIC
    soft_iic_init(&dl1b_iic_struct, DL1B_DEV_ADDR, DL1B_SOFT_IIC_DELAY, DL1B_SCL_PIN, DL1B_SDA_PIN);
#else
    iic_init(DL1B_IIC, DL1B_DEV_ADDR, DL1B_IIC_SPEED, DL1B_SCL_PIN, DL1B_SDA_PIN);
#endif
    gpio_init(DL1B_XS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    do
    {
        system_delay_ms(50);
        gpio_low(DL1B_XS_PIN);
        system_delay_ms(10);
        gpio_high(DL1B_XS_PIN);
        system_delay_ms(50);

        data_buffer[0] = DL1B_FIRMWARE__SYSTEM_STATUS >> 8;
        data_buffer[1] = DL1B_FIRMWARE__SYSTEM_STATUS & 0xFF;
        dl1b_transfer_8bit_array(data_buffer, 2, &data_buffer[2], 1);
        return_state = (0x01 == (data_buffer[2] & 0x01)) ? (0) : (1);
        if(1 == return_state)
        {
            break;
        }

        data_buffer[0] = DL1B_I2C_SLAVE__DEVICE_ADDRESS >> 8;
        data_buffer[1] = DL1B_I2C_SLAVE__DEVICE_ADDRESS & 0xFF;
        memcpy(&data_buffer[2], (uint8 *)dl1b_config_file, sizeof(dl1b_config_file));

        // VL53L1X Ĭ�� dl1b_config_file ���õ��� SHORT ģʽ (���� ~1.36m),
        // ����� dl1b_distance_mm > 1363 ������ 8192 �ĸ��� (range_status ���� valid)��
        // �� burst write ǰֱ�Ӹ� RAM ������� 12 �������ֽ�, �����ű��ĳ� LONG ģʽ
        // (���� ~3.6m, 33ms timing budget), sensor �յ� burst write ���һ֡���� LONG��
        // data_buffer[2] ��Ӧ�Ĵ��� 0x0001, �ʼĴ��� 0xRRRR ��ƫ���� 1 + 0xRRRR��
        // �ֽ�ֵ��Դ: ST ULD STSW-IMG009 �� VL53L1X_SetDistanceMode(LONG) +
        // VL53L1X_SetTimingBudgetInMs(33) �� LONG ģʽ�µı�׼������
        data_buffer[1 + 0x004B] = 0x0A;     // PHASECAL_CONFIG__TIMEOUT_MACROP
        data_buffer[1 + 0x005E] = 0x00;     // RANGE_CONFIG__TIMEOUT_MACROP_A_HI (33ms)
        data_buffer[1 + 0x005F] = 0x60;     // RANGE_CONFIG__TIMEOUT_MACROP_A_LO (33ms)
        data_buffer[1 + 0x0060] = 0x0F;     // RANGE_CONFIG__VCSEL_PERIOD_A
        data_buffer[1 + 0x0061] = 0x00;     // RANGE_CONFIG__TIMEOUT_MACROP_B_HI (33ms)
        data_buffer[1 + 0x0062] = 0x6E;     // RANGE_CONFIG__TIMEOUT_MACROP_B_LO (33ms)
        data_buffer[1 + 0x0063] = 0x0D;     // RANGE_CONFIG__VCSEL_PERIOD_B
        data_buffer[1 + 0x0069] = 0xB8;     // RANGE_CONFIG__VALID_PHASE_HIGH
        data_buffer[1 + 0x0078] = 0x0F;     // SD_CONFIG__WOI_SD0
        data_buffer[1 + 0x0079] = 0x0D;     // SD_CONFIG__WOI_SD1
        data_buffer[1 + 0x007A] = 0x0E;     // SD_CONFIG__INITIAL_PHASE_SD0
        data_buffer[1 + 0x007B] = 0x0E;     // SD_CONFIG__INITIAL_PHASE_SD1

        dl1b_transfer_8bit_array(data_buffer, 2 + sizeof(dl1b_config_file), data_buffer, 0);

        while(1)
        {
            data_buffer[0] = DL1B_GPIO__TIO_HV_STATUS >> 8;
            data_buffer[1] = DL1B_GPIO__TIO_HV_STATUS & 0xFF;
            dl1b_transfer_8bit_array(data_buffer, 2, &data_buffer[2], 1);
            if(0x00 == (data_buffer[2] & 0x01))
            {
                time_out_count = 0;
                break;
            }
            if(DL1B_TIMEOUT_COUNT < time_out_count ++)
            {
                return_state = 1;
                break;
            }
            system_delay_ms(1);
        }

        dl1b_init_flag = 1;
    }while(0);

#if DL1B_INT_ENABLE
    exti_init(DL1B_INT_PIN, EXTI_TRIGGER_FALLING);
    dl1b_int_handler();
    dl1b_finsh_flag = 0;
#endif
    set_tof_type(TOF_DL1B, dl1b_int_handler);

    return return_state;
}
