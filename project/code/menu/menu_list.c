#include "menu.h"
#include "zf_common_headfile.h"
#include "flight_control.h"
#include "parameter_flash.h"
#include "screen_pages.h"
#include "../icm/icm.h"
#include "../vofa.h"

/* 主菜单对象 */
Menu mainMenu;

/*
 * [2026-03-10]
 * 修改目的:
 * 1. 在菜单启动时尽早检查参数索引冲突，避免运行过程中再暴露这类配置错误。
 * 2. 保持提示逻辑集中在菜单入口附近，方便统一维护。
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

/* 控制模式选择页流程 */
static uint8 select_control_mode(void)
{
    uint8 selection = (Flight_Control_Get_Mode() == CONTROL_MODE_ANGLE) ? 1U : 0U;

    ui_menu_draw_control_mode_page(selection);
    key_flag = 0;

    while(1)
    {
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
/* 退出菜单系统并切入飞行监视页 */
void exit_menu_system(void)
{
    // 先让用户确认当前飞控模式。
    if (!select_control_mode())
        return;  // 用户取消操作
    
    // 播放退出动画，保持原有过渡体验。
    ui_menu_play_exit_animation();

    /*
     * [2026-03-10]
     * 修改目的:
     * 旧逻辑通过 g_menu_exit_flag 跳出菜单并在上层循环中进入飞行显示页，
     * 这里改为直接切到监视页状态，保持主流程入口不变但减少状态混用。
     */
    menu_enter_monitor_view();
    key_flag = 0;
}

void icm_init_menu()
{
    /*
     * [2026-03-10]
     * 修改目的:
     * 1. 保留主菜单里的 ICM 初始化入口，方便单独检查姿态解算与陀螺仪状态。
     * 2. 初始化完成后继续停留在结果页，直到用户主动返回，保持与菜单其他页面一致的交互方式。
     */
    key_flag = 0;
    IMU_calibrate_flag = 0;
    ICM_Calculate_Init();

    ips200_clear();
    while(1)
    {
        ui_menu_show_icm_init_result_page();
        if(key_flag == Button_4)
        {
            key_flag = Button_4;
            return;
        }

        system_delay_ms(20);
    }
}

/*
 * 菜单内容初始化:
 * 在这里集中注册主菜单项，方便统一查看当前菜单都暴露了哪些入口。
 */
void init_test_menu(void)
{
    // 初始化主菜单对象
    strncpy(mainMenu.title, "Main Menu", MAX_MENU_NAME_LEN-1);
    mainMenu.title[MAX_MENU_NAME_LEN - 1] = '\0';
    mainMenu.itemCount = 0;
    mainMenu.currentSelection = 0;
    mainMenu.parentMenu = NULL;
    
    // 注册主菜单项
    menu_create_item(&mainMenu, "Parameters", parameter_menu_entry, NULL);
    menu_create_item(&mainMenu, "ICM Init", icm_init_menu, NULL);
    menu_create_item(&mainMenu, "Flight View", exit_menu_system, NULL);
    
}

/*
 * 菜单启动前初始化:
 * 当前主要负责恢复菜单页状态，并在启动前给出必要的配置告警。
 */
void menu_open_init()
{
    /*
     * [2026-03-10]
     * 修改目的:
     * 上电后明确从主菜单界面起步，避免菜单状态残留到飞行监视页。
     */
    menu_reset_exit_flag();
    menu_enter_main_menu_view();
    show_parameter_conflict_warning();
}

/*
 * 菜单运行入口:
 * 负责初始化菜单对象、进入菜单循环，并在循环里同时处理按键和串口调参命令。
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
            // 菜单系统已经退出
            break;
        }
        
        // 轻微延时，避免菜单循环过于紧凑
        system_delay_ms(10);
    }
    
    if (!menu_is_monitor_view())
    {
        // 退出菜单后清屏，避免残留旧页面内容
        ips200_clear();
    }
}
