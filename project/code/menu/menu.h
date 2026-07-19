#ifndef __MENU_H__
#define __MENU_H__

#include "zf_common_headfile.h"
#include "key.h"

// ==================== 菜单容量限制 ====================
#define MAX_MENU_ITEMS 12
#define MAX_MENU_NAME_LEN 20

// 单个菜单项，可以绑定执行函数，也可以指向子菜单。
typedef struct MenuItem {
    char name[MAX_MENU_NAME_LEN];   // 菜单项名称
    void (*function)(void);         // 选中后执行的函数
    struct Menu* subMenu;           // 子菜单入口，NULL 表示当前项直接执行函数
} MenuItem;

// 菜单页对象，保存标题、菜单项和当前选中位置。
typedef struct Menu {
    char title[MAX_MENU_NAME_LEN];  // 菜单标题
    MenuItem items[MAX_MENU_ITEMS]; // 菜单项数组
    uint8 itemCount;                // 已创建的菜单项数量
    uint8 currentSelection;         // 当前高亮的菜单项索引
    struct Menu* parentMenu;        // 上一级菜单
} Menu;

/*
 * [2026-03-10]
 * 修改目的:
 * 1. 把“菜单界面”和“飞行监视界面”拆成显式状态，避免继续复用 g_menu_exit_flag。
 * 2. 在不改 menu_open/menu_open_init 等主流程接口名称的前提下，补一个更稳定的 UI 状态机。
 */
typedef enum MenuScreenState {
    MENU_SCREEN_MENU = 0,
    MENU_SCREEN_MONITOR = 1
} MenuScreenState;

// 当前显示的活动菜单。
extern Menu* g_currentMenu;

// 菜单退出标志，置 1 后上层流程会退出菜单循环。
extern uint8 g_menu_exit_flag;

// 当前界面状态: 菜单列表 or 飞行监视页。
extern volatile uint8 g_menu_screen_state;

// 初始化菜单显示环境。
void menu_init(void);

// 把指定菜单完整绘制到屏幕上。
void menu_display(Menu* menu);

// 处理一次菜单输入与状态更新，返回当前有效菜单。
Menu* menu_process(Menu* currentMenu);

// 只刷新选中项变化，避免整页重绘。
void menu_update_selection(Menu* menu, uint8 old_selection, uint8 new_selection);

// 按键分发入口，负责上下移动、进入和返回。
void menu_key_handler(Menu* menu, uint8 key);

// 向菜单追加一个菜单项。
void menu_create_item(Menu* menu, const char* name, void (*function)(void), Menu* subMenu);

// 切换当前活动菜单并刷新显示。
void menu_set_current(Menu* menu);

// 获取当前活动菜单指针。
Menu* menu_get_current(void);

// 从当前菜单进入选中的子菜单。
Menu* menu_enter_submenu(Menu* currentMenu);

// 返回当前菜单的父菜单。
Menu* menu_return_parent(Menu* currentMenu);

// 执行当前选中项对应的函数或进入子菜单。
void menu_execute_function(Menu* menu);

// 请求退出菜单系统。
void menu_exit(void);

// 清除退出标志，重新允许菜单运行。
void menu_reset_exit_flag(void);

// 进入飞行监视界面。
void menu_enter_monitor_view(void);

// 回到主菜单界面。
void menu_enter_main_menu_view(void);

// 查询当前是否处于飞行监视界面。
uint8 menu_is_monitor_view(void);

#endif /* __MENU_H__ */
