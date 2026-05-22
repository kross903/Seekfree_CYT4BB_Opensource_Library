/*********************************************************************************************************************
* CYT4BB Opensourec Library ���� CYT4BB ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� CYT4BB ��Դ���һ����
*
* CYT4BB ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          main_cm7_1
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "../code/common/project_headfile.h"

static void camera_core_init_vision_modules(void)
{
    IR_Localization_Init();
    BeaconDetect_Init();
    IPM_Interface_Init();
}

static void camera_core_service_lost_timer(void)
{
    static uint8 loop_divider = 0U;
    VisionRuntimeDiag_t vision_diag;
    CrossCoreVisionControlResult_t control_result;

    loop_divider++;
    if(loop_divider >= 5U)
    {
        loop_divider = 0U;
        IR_Localization_Task_5ms();
        VisionRuntime_UpdateDiagFromCurrentState();
        VisionRuntime_GetDiag(&vision_diag);
        CrossCoreVision_BuildControlResult(&ir_pose_estimate,
                                           ir_position_lost_flag,
                                           &vision_diag,
                                           &control_result);
        CrossCoreVision_CameraCorePublishOutput(&control_result, &vision_diag);
    }
}

static void camera_core_process_vision_frame(void)
{
    static CrossCoreVisionInput_t vision_input = {0U, 0.0f, 0.0f, 0.0f, 100.0f, 0U};
    CrossCoreVisionInput_t latest_input;
    VisionRuntimeDiag_t vision_diag;
    CrossCoreVisionControlResult_t control_result;

    if(CrossCoreVision_CameraCoreFetchInput(&latest_input))
    {
        vision_input = latest_input;
    }

    if(CrossCoreCamera_CameraCoreTask())
    {
        VisionRuntime_ProcessImage(mt9v03x_image,
                                   vision_input.roll_deg,
                                   vision_input.pitch_deg,
                                   vision_input.height_cm);
        VisionRuntime_GetDiag(&vision_diag);
        CrossCoreVision_BuildControlResult(&ir_pose_estimate,
                                           ir_position_lost_flag,
                                           &vision_diag,
                                           &control_result);
        CrossCoreVision_CameraCorePublishOutput(&control_result, &vision_diag);
    }
}

// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������

// **************************** �������� ****************************

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// ʱ�����ü�ϵͳ��ʼ��<��ر���>
    debug_init();                       // ���Դ�����Ϣ��ʼ��
     
    // �˴���д�û����� ���������ʼ�������

    CrossCoreCamera_CameraCoreInit();
    CrossCoreVision_InitShared();
    camera_core_init_vision_modules();
    

    // �˴���д�û����� ���������ʼ�������
    while(true)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���
        camera_core_service_lost_timer();
        camera_core_process_vision_frame();
        system_delay_ms(1);

      
      
        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}

// **************************** �������� ****************************
