#include "common/project_headfile.h"

/* ????? */
Menu mainMenu;
static Menu testMenu;

/*
 * [2026-03-10]
 * 修改目的:
 * 1. ??????????????????????????????????
 * 2. ???????????????????????
 */
static void show_parameter_conflict_warning(void)
{
    if (!check_parameter_index_conflicts())
    {
        return;
    }

    ui_menu_show_parameter_conflict_warning();
    system_delay_ms(800);
}

/* ????????? */
static uint8 select_control_mode(void)
{
    uint8 selection = (Flight_Control_Get_Mode() == CONTROL_MODE_ANGLE) ? 1U : 0U;

    ui_menu_draw_control_mode_page(selection);
    key_flag = 0;

    while(1)
    {
        core_link_user_poll();

        if(key_flag != 0)
        {
            switch(key_flag)
            {
                case Button_1:
                case Button_2:
                    selection = selection ? 0U : 1U;
                    ui_menu_draw_control_mode_page(selection);
                    break;

                case Button_3:
                    Flight_Control_Set_Mode(selection);
                    ui_menu_show_control_mode_saved();
                    system_delay_ms(500);
                    return 1U;

                case Button_4:
                    return 0U;

                default:
                    break;
            }
            key_flag = 0;
        }
    }
}
/* ?????????????? */
void exit_menu_system(void)
{
    // ?????????????
    if (!select_control_mode())
        return;  // 用户取消操作
    
    // ????????????????
    ui_menu_play_exit_animation();

    /*
     * [2026-03-10]
     * 修改目的:
     * ????? g_menu_exit_flag ???????????????????
     * ???????????????????????????????
     */
    menu_enter_monitor_view();
    key_flag = 0;
}

void icm_init_menu()
{
    /*
     * [2026-03-10]
     * 修改目的:
     * 1. ??????? ICM ???????????????????????
     * 2. ?????????????????????????????????????????
     */
    key_flag = 0;
    IMU_calibrate_flag = 0;
    ICM_Calculate_Init();

    ips200_clear();
    while(1)
    {
        core_link_user_poll();
        ui_menu_show_icm_init_result_page();
        if(key_flag == Button_4)
        {
            key_flag = Button_4;
            return;
        }

        system_delay_ms(20);
    }
}

static void test_placeholder_page(void)
{
    ips200_show_string(20U, 72U, "Reserved test slot");
    ips200_show_string(20U, 96U, "Add test code here");
    ips200_show_string(20U, 120U, "Use KEY4 to return");
}

static void motor_test_page(void)
{
    static uint8 edit_field = 0U;
    uint8 selected_motor = 0U;
    uint8 output_enable = 0U;
    uint16 throttle = 0U;
    uint16 output_limit = 0U;
    uint16 test_limit = 0U;

    if(!Flight_Motor_Test_Is_Active())
    {
        Flight_Motor_Test_Start();
        edit_field = 0U;
    }

    if(key_flag == Button_4)
    {
        Flight_Motor_Test_Stop();
        return;
    }

    selected_motor = Flight_Motor_Test_Get_Selected();
    throttle = Flight_Motor_Test_Get_Throttle();
    output_enable = Flight_Motor_Test_Get_Output_Enable();
    output_limit = Flight_Motor_Test_Get_Output_Limit();
    test_limit = (MOTOR_TEST_MAX_THROTTLE < output_limit) ? MOTOR_TEST_MAX_THROTTLE : output_limit;

    if(key_flag == Button_1)
    {
        switch(edit_field)
        {
            case 0U:
                selected_motor = (selected_motor == 0U) ? 3U : (selected_motor - 1U);
                Flight_Motor_Test_Set_Selected(selected_motor);
                break;

            case 1U:
                throttle = (throttle > MOTOR_TEST_STEP) ? (throttle - MOTOR_TEST_STEP) : 0U;
                Flight_Motor_Test_Set_Throttle(throttle);
                break;

            case 2U:
                Flight_Motor_Test_Set_Output_Enable(0U);
                break;

            default:
                break;
        }
        key_flag = 0;
    }
    else if(key_flag == Button_2)
    {
        switch(edit_field)
        {
            case 0U:
                selected_motor = (selected_motor + 1U) & 0x03U;
                Flight_Motor_Test_Set_Selected(selected_motor);
                break;

            case 1U:
                throttle = (throttle + MOTOR_TEST_STEP > test_limit) ? test_limit : (throttle + MOTOR_TEST_STEP);
                Flight_Motor_Test_Set_Throttle(throttle);
                break;

            case 2U:
                Flight_Motor_Test_Set_Output_Enable(1U);
                break;

            default:
                break;
        }
        key_flag = 0;
    }
    else if(key_flag == Button_3)
    {
        edit_field = (edit_field + 1U) % 3U;
        key_flag = 0;
    }

    selected_motor = Flight_Motor_Test_Get_Selected();
    throttle = Flight_Motor_Test_Get_Throttle();
    output_enable = Flight_Motor_Test_Get_Output_Enable();

    ips200_show_string(20U, 48U, "K3:NEXT FIELD      ");
    ips200_show_string(20U, 64U, (edit_field == 0U) ? "> MOTOR:" : "  MOTOR:");
    ips200_show_string(140U, 64U, "M");
    ips200_show_int(156U, 64U, (int)(selected_motor + 1U), 1U);

    ips200_show_string(20U, 88U, (edit_field == 1U) ? "> THR:" : "  THR:");
    ips200_show_int(140U, 88U, (int)throttle, 4U);

    ips200_show_string(20U, 112U, (edit_field == 2U) ? "> OUT:" : "  OUT:");
    ips200_show_string(140U, 112U, output_enable ? "ON " : "OFF");

    ips200_show_string(20U, 136U, "TEST MAX:");
    ips200_show_int(140U, 136U, (int)test_limit, 4U);
    ips200_show_string(20U, 160U, "OUT LIM:");
    ips200_show_int(140U, 160U, (int)output_limit, 4U);
    ips200_show_string(20U, 184U, "SCALE:");
    ips200_show_int(140U, 184U, (int)MOTOR_OUTPUT_SCALE_MAX, 4U);

    ips200_show_string(20U, 208U, "M1:");
    ips200_show_int(52U, 208U, (int)motor_throttle[0], 4U);
    ips200_show_string(120U, 208U, "M2:");
    ips200_show_int(152U, 208U, (int)motor_throttle[1], 4U);
    ips200_show_string(20U, 232U, "M3:");
    ips200_show_int(52U, 232U, (int)motor_throttle[2], 4U);
    ips200_show_string(120U, 232U, "M4:");
    ips200_show_int(152U, 232U, (int)motor_throttle[3], 4U);
}

/*
 * ????????
 * ???????????????????????????????
 */
void init_test_menu(void)
{
    // 初始化主菜单对象
    strncpy(mainMenu.title, "Main Menu", MAX_MENU_NAME_LEN-1);
    mainMenu.title[MAX_MENU_NAME_LEN - 1] = '\0';
    mainMenu.itemCount = 0;
    mainMenu.currentSelection = 0;
    mainMenu.parentMenu = NULL;

    strncpy(testMenu.title, "Test Menu", MAX_MENU_NAME_LEN - 1);
    testMenu.title[MAX_MENU_NAME_LEN - 1] = '\0';
    testMenu.itemCount = 0;
    testMenu.currentSelection = 0;
    testMenu.parentMenu = NULL;
    
    // 注册主菜单项
    menu_create_item(&mainMenu, "Parameters", parameter_menu_entry, NULL);
    menu_create_item(&mainMenu, "Test", NULL, &testMenu);
    menu_create_item(&mainMenu, "ICM Init", icm_init_menu, NULL);
    menu_create_item(&mainMenu, "Flight View", exit_menu_system, NULL);

    menu_create_item(&testMenu, "IMU Test", test_placeholder_page, NULL);
    menu_create_item(&testMenu, "PMW Test", test_placeholder_page, NULL);
    menu_create_item(&testMenu, "Motor Test", motor_test_page, NULL);
    menu_create_item(&testMenu, "VOFA Test", test_placeholder_page, NULL);
    
}

/*
 * 菜单启动前初始化:
 * ?????????????????????????????
 */
void menu_open_init()
{
    /*
     * [2026-03-10]
     * 修改目的:
     * ?????????????????????????????
     */
    menu_reset_exit_flag();
    menu_enter_main_menu_view();
    show_parameter_conflict_warning();
}

/*
 * 菜单运行入口:
 * ????????????????????????????????????
 */
void menu_open(void)
{
    if (menu_is_monitor_view())
    {
        return;
    }

    menu_init();
    menu_reset_exit_flag();
    menu_enter_main_menu_view();
    init_test_menu();
    menu_set_current(&mainMenu);
    
    // 菜单处理循环
    while (!g_menu_exit_flag && !menu_is_monitor_view())
    {
        // 处理菜单逻辑
        g_currentMenu = menu_process(g_currentMenu);
        Send_test();
        USART_PID_Adjust();
        if (g_currentMenu == NULL)
        {
            // ?????????
            break;
        }
        
        // ????????????????
        system_delay_ms(10);
    }
    
    if (!menu_is_monitor_view())
    {
        // 退出菜单后清屏，避免残留旧页面内容
        ips200_clear();
    }
}
