#include "icm42688.h"
#include "common/project_headfile.h"
#include <math.h>

int icm_calibration_over_flag = 0;
int icm_calibration_flag = 0;
// ICM42688加速度计数据
float icm42688_acc_x  = 0, icm42688_acc_y  = 0, icm42688_acc_z  = 0;
float icm42688_acc_x_use  = 0, icm42688_acc_y_use  = 0, icm42688_acc_z_use  = 0;
short int icm42688_acc_x_r = 0, icm42688_acc_y_r = 0, icm42688_acc_z_r = 0;
float acc = 0;
float acc_use = 0;
// ICM42688角加速度数据
float icm42688_gyro_x = 0, icm42688_gyro_y = 0, icm42688_gyro_z = 0;
short int icm42688_gyro_x_r = 0, icm42688_gyro_y_r = 0, icm42688_gyro_z_r = 0;
float prev_gx = 0.0f, prev_gy = 0.0f, prev_gz = 0.0f;   // 上一次角速度 (rad/s)
float alpha_gx = 0.0f, alpha_gy = 0.0f, alpha_gz = 0.0f; // 当前角加速度 (rad/s?)
float gyro_x_correction, gyro_y_correction, gyro_z_correction;//零漂修正
float gyro_x_correction_experience, gyro_y_correction_experience, gyro_z_correction_experience;//零漂修正经验值
short int IMU_calibrate_flag = 0;//0的时候开启陀螺仪测定零飘矫正，1的时候用经验值矫正

// SPI协议读写操作宏定义
#define ICM42688_Write_Reg(reg, data)       spi_write_8bit_register(ICM42688_SPI, reg, data);
#define ICM42688_Read_Regs(reg, data,num)   spi_read_8bit_registers(ICM42688_SPI, reg | 0x80, data, num);

// 静态函数声明,以下函数均为该.c文件内部调用
static void Write_Data_ICM42688(unsigned char reg, unsigned char data);
static void Read_Datas_ICM42688(unsigned char reg, unsigned char *data, unsigned int num);
// 数据转换为实际物理数据的转换系数
float icm42688_acc_inv = 1, icm42688_gyro_inv = 1;

//void SPI_switch_to_ICM42688(void){
//    ((SPI_TypeDef *)SPI3_BASE)->CTLR1 &= 0xFFF4;// 18M MODE0
//}

/**
*
* @brief    ICM42688陀螺仪初始化
* @param
* @return   void
* @notes    用户调用
* Example:  Init_ICM42688();
*
**/
void Init_ICM42688(void)
{
    // SPI初始化
     system_delay_ms(10);
    spi_init(ICM42688_SPI, SPI_MODE0, ICM42688_SPI_SPEED, ICM42688_SPC_PIN, ICM42688_SDI_PIN, ICM42688_SDO_PIN, SPI_CS_NULL);
    gpio_init(ICM42688_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    // [2026-03-10] ICM init UI is now drawn by icm.c, so suppress the old
    // left-top success prompts here to avoid overwriting the stage page.
    // 初始化超时
    int32 time = 50;
    // 读取陀螺仪型号陀螺仪自检
    unsigned char model = 0xff;
    while(1)
    {
        // 读芯片ID
        Read_Datas_ICM42688(ICM42688_WHO_AM_I, &model, 1);
//        printf("%x\n",model);
        if(model == 0x47)
        {
            // ICM42688,71
//            printf("ok");
            break;
        }
        else
        {
            ICM42688_DELAY_MS(10);
            time--;
            if(time < 0)
            {
                zf_log(0, "ICM42688 Init Error!");
                ips200_show_string(0, 0, "ICM42688 Init Error!");
                while(1);
                // 卡在这里原因有以下几点
                // ICM42688坏了,如果是新的概率极低
                // 接线错误或者没有接好
                // 接线太长,通信失败
            }
        }
    }
    Write_Data_ICM42688(ICM42688_PWR_MGMT0, 0x00);      // 复位设备
    ICM42688_DELAY_MS(20);                              // 操作完PWR—MGMT0寄存器后200us内不能有任何读写寄存器的操作


    Set_LowpassFilter_Range_ICM42688(ICM42688_AFS_16G, ICM42688_AODR_8000HZ,
                                      ICM42688_GFS_2000DPS, ICM42688_GODR_8000HZ);


    Write_Data_ICM42688(ICM42688_PWR_MGMT0, 0x0f);      // 设置GYRO_MODE,ACCEL_MODE为低噪声模式
    ICM42688_DELAY_MS(10);
}

/**
*
* @brief    获得ICM42688陀螺仪加速度
* @param
* @return   void
* @notes    单位:g(m/s^2),用户调用
* Example:  Get_Acc_ICM42688();
*
**/
void Get_Acc_ICM42688(void)
{
    unsigned char data[6];
    short int raw_x_r;
    short int raw_y_r;
    short int raw_z_r;
    float raw_x;
    float raw_y;
    float raw_z;
    float roll_rad;
    float pitch_rad;
    float sin_roll;
    float cos_roll;
    float sin_pitch;
    float cos_pitch;
    float roll_y;
    float roll_z;

    Read_Datas_ICM42688(ICM42688_ACCEL_DATA_X1, data, 6);
    raw_x_r = (short int)(((short int)data[0] << 8) | data[1]);
    raw_y_r = (short int)(((short int)data[2] << 8) | data[3]);
    raw_z_r = (short int)(((short int)data[4] << 8) | data[5]);
    icm42688_acc_x_r = raw_x_r;
    icm42688_acc_y_r = raw_y_r;
    icm42688_acc_z_r = raw_z_r;

    raw_x = icm42688_acc_inv * raw_x_r;
    raw_y = icm42688_acc_inv * raw_y_r;
    raw_z = icm42688_acc_inv * raw_z_r;

    /*
     * Apply fixed sensor-to-body installation compensation before the data
     * enters the attitude solver. We currently compensate roll and pitch only.
     */
    roll_rad = angle_roll_compensation * DEG2RAD;
    pitch_rad = angle_pitch_compensation * DEG2RAD;
    sin_roll = sinf(roll_rad);
    cos_roll = cosf(roll_rad);
    sin_pitch = sinf(pitch_rad);
    cos_pitch = cosf(pitch_rad);

    roll_y = cos_roll * raw_y - sin_roll * raw_z;
    roll_z = sin_roll * raw_y + cos_roll * raw_z;

    icm42688_acc_x = cos_pitch * raw_x + sin_pitch * roll_z;
    icm42688_acc_y = roll_y;
    icm42688_acc_z = -sin_pitch * raw_x + cos_pitch * roll_z;
}

/**
*
* @brief    获得ICM42688陀螺仪角加速度
* @param
* @return   void
* @notes    单位为:°/s,用户调用
* Example:  Get_Gyro_ICM42688();
*
**/
void Get_RAW_Acc_ICM42688(void)
{
    unsigned char data[6];
    Read_Datas_ICM42688(ICM42688_ACCEL_DATA_X1, data, 6);
    icm42688_acc_x_r = (short int)(((short int)data[0] << 8) | data[1]);
    icm42688_acc_y_r = (short int)(((short int)data[2] << 8) | data[3]);
    icm42688_acc_z_r = (short int)(((short int)data[4] << 8) | data[5]);
}

void Get_correction(void)
{
    unsigned char data[6];
    Read_Datas_ICM42688(ICM42688_GYRO_DATA_X1, data, 6);
    icm42688_gyro_x = icm42688_gyro_inv * (short int)(((short int)data[0] << 8) | data[1]);
    icm42688_gyro_y = icm42688_gyro_inv * (short int)(((short int)data[2] << 8) | data[3]);
    icm42688_gyro_z = icm42688_gyro_inv * (short int)(((short int)data[4] << 8) | data[5]);
}
void Get_Gyro_ICM42688(void)
{
    unsigned char data[6];
    short int raw_x_r;
    short int raw_y_r;
    short int raw_z_r;
    float raw_x;
    float raw_y;
    float raw_z;
    float roll_rad;
    float pitch_rad;
    float sin_roll;
    float cos_roll;
    float sin_pitch;
    float cos_pitch;
    float roll_y;
    float roll_z;

    Read_Datas_ICM42688(ICM42688_GYRO_DATA_X1, data, 6);
    raw_x_r = (short int)(((short int)data[0] << 8) | data[1]);
    raw_y_r = (short int)(((short int)data[2] << 8) | data[3]);
    raw_z_r = (short int)(((short int)data[4] << 8) | data[5]);
    icm42688_gyro_x_r = raw_x_r;
    icm42688_gyro_y_r = raw_y_r;
    icm42688_gyro_z_r = raw_z_r;

    raw_x = icm42688_gyro_inv * raw_x_r - gyro_x_correction;
    raw_y = icm42688_gyro_inv * raw_y_r - gyro_y_correction;
    raw_z = icm42688_gyro_inv * raw_z_r - gyro_z_correction;

    roll_rad = angle_roll_compensation * DEG2RAD;
    pitch_rad = angle_pitch_compensation * DEG2RAD;
    sin_roll = sinf(roll_rad);
    cos_roll = cosf(roll_rad);
    sin_pitch = sinf(pitch_rad);
    cos_pitch = cosf(pitch_rad);

    roll_y = cos_roll * raw_y - sin_roll * raw_z;
    roll_z = sin_roll * raw_y + cos_roll * raw_z;

    icm42688_gyro_x = cos_pitch * raw_x + sin_pitch * roll_z;
    icm42688_gyro_y = roll_y;
    icm42688_gyro_z = -sin_pitch * raw_x + cos_pitch * roll_z;
}
void Get_RAW_Gyro_ICM42688(void)
{
    unsigned char data[6];
    Read_Datas_ICM42688(ICM42688_GYRO_DATA_X1, data, 6);
    icm42688_gyro_x_r = (short int)(((short int)data[0] << 8) | data[1]);
    icm42688_gyro_y_r = (short int)(((short int)data[2] << 8) | data[3]);
    icm42688_gyro_z_r = (short int)(((short int)data[4] << 8) | data[5]);
}

/**
*
* @brief    设置ICM42688陀螺仪低通滤波器带宽和量程
* @param    afs                 // 加速度计量程,可在dmx_icm42688.h文件里枚举定义中查看
* @param    aodr                // 加速度计输出速率,可在dmx_icm42688.h文件里枚举定义中查看
* @param    gfs                 // 陀螺仪量程,可在dmx_icm42688.h文件里枚举定义中查看
* @param    godr                // 陀螺仪输出速率,可在dmx_icm42688.h文件里枚举定义中查看
* @return   void
* @notes    ICM42688.c文件内部调用,用户无需调用尝试
* Example:  Set_LowpassFilter_Range_ICM42688(ICM42688_AFS_16G,ICM42688_AODR_32000HZ,ICM42688_GFS_2000DPS,ICM42688_GODR_32000HZ);
*
**/
void Set_LowpassFilter_Range_ICM42688(enum icm42688_afs afs, enum icm42688_aodr aodr, enum icm42688_gfs gfs, enum icm42688_godr godr)
{
    Write_Data_ICM42688(ICM42688_ACCEL_CONFIG0, (afs << 5) | (aodr + 1));   // 初始化ACCEL量程和输出速率(p77)
    Write_Data_ICM42688(ICM42688_GYRO_CONFIG0, (gfs << 5) | (godr + 1));    // 初始化GYRO量程和输出速率(p76)

    switch(afs)
    {
    case ICM42688_AFS_2G:
        icm42688_acc_inv = 2000 / 32768.0f;             // 加速度计量程为:±2g
        break;
    case ICM42688_AFS_4G:
        icm42688_acc_inv = 4000 / 32768.0f;             // 加速度计量程为:±4g
        break;
    case ICM42688_AFS_8G:
        icm42688_acc_inv = 8000 / 32768.0f;             // 加速度计量程为:±8g
        break;
    case ICM42688_AFS_16G:
        icm42688_acc_inv = 16000 / 32768.0f;            // 加速度计量程为:±16g
        break;
    default:
        icm42688_acc_inv = 1;                           // 不转化为实际数据
        break;
    }
    switch(gfs)
    {
    case ICM42688_GFS_15_625DPS:
        icm42688_gyro_inv = 15.625f / 32768.0f;         // 陀螺仪量程为:±15.625dps
        break;
    case ICM42688_GFS_31_25DPS:
        icm42688_gyro_inv = 31.25f / 32768.0f;          // 陀螺仪量程为:±31.25dps
        break;
    case ICM42688_GFS_62_5DPS:
        icm42688_gyro_inv = 62.5f / 32768.0f;           // 陀螺仪量程为:±62.5dps
        break;
    case ICM42688_GFS_125DPS:
        icm42688_gyro_inv = 125.0f / 32768.0f;          // 陀螺仪量程为:±125dps
        break;
    case ICM42688_GFS_250DPS:
        icm42688_gyro_inv = 250.0f / 32768.0f;          // 陀螺仪量程为:±250dps
        break;
    case ICM42688_GFS_500DPS:
        icm42688_gyro_inv = 500.0f / 32768.0f;          // 陀螺仪量程为:±500dps
        break;
    case ICM42688_GFS_1000DPS:
        icm42688_gyro_inv = 1000.0f / 32768.0f;         // 陀螺仪量程为:±1000dps
        break;
    case ICM42688_GFS_2000DPS:
        icm42688_gyro_inv = 2000.0f / 32768.0f;         // 陀螺仪量程为:±2000dps
        break;
    default:
        icm42688_gyro_inv = 1;                          // 不转化为实际数据
        break;
    }
}

/**
*
* @brief    ICM42688陀螺仪写数据
* @param    reg                 寄存器
* @param    data                需要写进该寄存器的数据
* @return   void
* @notes    ICM42688.c文件内部调用,用户无需调用尝试
* Example:  Write_Data_ICM42688(0x00,0x00);
*
**/
static void Write_Data_ICM42688(unsigned char reg, unsigned char data)
{
    ICM42688_CS_LEVEL(GPIO_LOW);
    ICM42688_Write_Reg(reg, data);
    ICM42688_CS_LEVEL(GPIO_HIGH);
}

/**
*
* @brief    ICM42688陀螺仪读数据
* @param    reg                 寄存器
* @param    data                把读出的数据存入data
* @param    num                 数据个数
* @return   void
* @notes    ICM42688.c文件内部调用,用户无需调用尝试
* Example:  Read_Datas_ICM42688(0x00,data,1);
*
**/
static void Read_Datas_ICM42688(unsigned char reg, unsigned char *data, unsigned int num)
{
    ICM42688_CS_LEVEL(GPIO_LOW);
    ICM42688_Read_Regs(reg, data, num);
    ICM42688_CS_LEVEL(GPIO_HIGH);
}

///**
// * @brief IMU校准函数
// *
// * 该函数用于对IMU（惯性测量单元）进行校准。校准过程中，系统会提示用户保持设备静止，
// * 并在指定时间内采集陀螺仪数据。采集到的数据用于计算陀螺仪的偏移校正值。
// *
// * 校准步骤：
// * 1. 显示校准提示信息，提示用户保持设备静止。
// * 2. 在指定时间内（IMU_calibration_seconds），以4KHz的采样率采集陀螺仪数据。
// * 3. 计算每秒钟的陀螺仪数据平均值，并存储在gyro_history数组中。
// * 4. 计算整个校准时间内的陀螺仪数据平均值，并根据该平均值计算陀螺仪的偏移校正值。
// * 5. 显示校准完成提示信息。
// */
//#define IMU_calibration_seconds 3
//void IMU_calibration(void){
//
//    uint32 s_cnt=0,g_cnt=0;
//
//    float gyro_history[3][IMU_calibration_seconds];
//    sint32 gyro_history_sum[3]={0,0,0};
//    //校准提示
//    ips200_show_string(0,16*0,"IMU calibration");
//    ips200_show_string(0,16*2,"Don't move me!(2)");system_delay_ms(1000);
//    ips200_show_string(0,16*2,"Don't move me!(1)");system_delay_ms(1000);
//    do{
//        system_delay_us(250);
//        Get_RAW_Gyro_ICM42688();
//        if(g_cnt<4000){
//            gyro_history_sum[0] += icm42688_gyro_x_r;
//            gyro_history_sum[1] += icm42688_gyro_y_r;
//            gyro_history_sum[2] += icm42688_gyro_z_r;
//            g_cnt++;
//        }
//        else{
//            gyro_history[0][s_cnt]=(float)gyro_history_sum[0]/4000.0f;
//            gyro_history[1][s_cnt]=(float)gyro_history_sum[1]/4000.0f;
//            gyro_history[2][s_cnt]=(float)gyro_history_sum[2]/4000.0f;
//            gyro_history_sum[0]=0;
//            gyro_history_sum[1]=0;
//            gyro_history_sum[2]=0;
//            g_cnt=0;
//            s_cnt++;
//        }
//    }while(s_cnt<IMU_calibration_seconds);
//
//
//    float sum[3]={0,0,0},k=icm42688_gyro_inv/IMU_calibration_seconds;
//    for(uint8 i=0;i<IMU_calibration_seconds;i++){
//        sum[0] += gyro_history[0][i];
//        sum[1] += gyro_history[1][i];
//        sum[2] += gyro_history[2][i];
//    }
//    gyro_x_correction = k*sum[0];
//    gyro_y_correction = k*sum[1];
//    gyro_z_correction = k*sum[2];
//    ips200_show_string(0,16*2,"IMU Calibration OK      ");
//
//}
