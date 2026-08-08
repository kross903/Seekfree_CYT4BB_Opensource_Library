#include "menu.h"
#include "common/project_headfile.h"

Menu* g_currentMenu = NULL;
uint8 g_menu_exit_flag = 0;
volatile uint8 g_menu_screen_state = MENU_SCREEN_MENU;

#define MENU_START_X       10
#define MENU_START_Y       40
#define MENU_ITEM_HEIGHT   20
#define MENU_ITEM_WIDTH    200
#define MENU_TITLE_Y       10

#define MENU_BG_COLOR      RGB565_BLACK
#define MENU_TEXT_COLOR    RGB565_WHITE
#define MENU_SELECT_COLOR  RGB565_BLUE
#define MENU_TITLE_COLOR   RGB565_YELLOW

void menu_init(void)
{
    ui_menu_init_display();
}

void menu_display(Menu* menu)
{
    ui_menu_display(menu);
}

Menu* menu_process(Menu* currentMenu)
{
    Menu* menu = currentMenu;

    core_link_user_poll();

    if(g_menu_exit_flag)
    {
        return NULL;
    }

    if(menu == NULL)
    {
        return NULL;
    }

    if(key_flag != 0)
    {
        menu_key_handler(menu, key_flag);
        key_flag = 0;
    }

    return g_currentMenu;
}

void menu_update_selection(Menu* menu, uint8 old_selection, uint8 new_selection)
{
    ui_menu_update_selection(menu, old_selection, new_selection);
}

void menu_key_handler(Menu* menu, uint8 key)
{
    uint8 old_selection = 0;

    if(menu == NULL || menu->itemCount == 0)
    {
        return;
    }

    if(menu->currentSelection >= menu->itemCount)
    {
        menu->currentSelection = 0;
    }

    old_selection = menu->currentSelection;

    switch(key)
    {
        case Button_1:
            if(menu->currentSelection > 0)
            {
                menu->currentSelection--;
            }
            else
            {
                menu->currentSelection = menu->itemCount - 1;
            }
            menu_update_selection(menu, old_selection, menu->currentSelection);
            break;

        case Button_2:
            if(menu->currentSelection < menu->itemCount - 1)
            {
                menu->currentSelection++;
            }
            else
            {
                menu->currentSelection = 0;
            }
            menu_update_selection(menu, old_selection, menu->currentSelection);
            break;

        case Button_3:
            menu_execute_function(menu);
            break;

        case Button_4:
            g_currentMenu = menu_return_parent(menu);
            break;

        default:
            break;
    }
}

void menu_create_item(Menu* menu, const char* name, void (*function)(void), Menu* subMenu)
{
    MenuItem* item = NULL;

    if(menu == NULL || menu->itemCount >= MAX_MENU_ITEMS)
    {
        return;
    }

    item = &menu->items[menu->itemCount];
    strncpy(item->name, name, MAX_MENU_NAME_LEN - 1);
    item->name[MAX_MENU_NAME_LEN - 1] = '\0';
    item->function = function;
    item->subMenu = subMenu;

    if(subMenu != NULL)
    {
        subMenu->parentMenu = menu;
    }

    menu->itemCount++;
}

void menu_set_current(Menu* menu)
{
    g_currentMenu = menu;
    menu_display(menu);
}

Menu* menu_get_current(void)
{
    return g_currentMenu;
}

Menu* menu_enter_submenu(Menu* currentMenu)
{
    MenuItem* item = NULL;

    if(currentMenu == NULL || currentMenu->itemCount == 0)
    {
        return NULL;
    }

    if(currentMenu->currentSelection >= currentMenu->itemCount)
    {
        currentMenu->currentSelection = 0;
    }

    item = &currentMenu->items[currentMenu->currentSelection];
    if(item->subMenu != NULL)
    {
        g_currentMenu = item->subMenu;
        g_currentMenu->currentSelection = 0;
        menu_display(g_currentMenu);
        return g_currentMenu;
    }

    return currentMenu;
}

Menu* menu_return_parent(Menu* currentMenu)
{
    if(currentMenu == NULL || currentMenu->parentMenu == NULL)
    {
        return currentMenu;
    }

    g_currentMenu = currentMenu->parentMenu;
    menu_display(g_currentMenu);
    return g_currentMenu;
}

void menu_execute_function(Menu* menu)
{
    MenuItem* item = NULL;
#if CY_CORE_CM7_0
    HeightTofSnapshot_t height_snapshot = {0};
#endif

    if(menu == NULL || menu->itemCount == 0)
    {
        return;
    }

    if(menu->currentSelection >= menu->itemCount)
    {
        menu->currentSelection = 0;
    }

    item = &menu->items[menu->currentSelection];

    if(item->subMenu != NULL)
    {
        g_currentMenu = menu_enter_submenu(menu);
        return;
    }

    if(item->function == NULL)
    {
        return;
    }

    ui_menu_draw_function_header(item->name);

    while(1)
    {
        core_link_user_poll();
#if CY_CORE_CM7_0
        CameraBoardLink_MainTask();
        HeightManager_GetTofSnapshot(&height_snapshot);
        (void)CrossCoreCamera_ControlCoreService(current_roll,
                                                 current_pitch,
                                                 current_yaw,
                                                 height_snapshot.filtered_height_cm,
                                                 height_snapshot.filtered_height_cm);
#endif
        item->function();

        if(g_menu_exit_flag || menu_is_monitor_view())
        {
            break;
        }

        if(key_flag == Button_4)
        {
            key_flag = 0;
            break;
        }

        system_delay_ms(5);
    }

    if(!g_menu_exit_flag && !menu_is_monitor_view())
    {
        menu_display(menu);
    }
}

void menu_exit(void)
{
    g_menu_exit_flag = 1;
}

void menu_reset_exit_flag(void)
{
    g_menu_exit_flag = 0;
}

/*
 * [2026-03-10]
 * 修改目的:
 * 1. 将“菜单列表”和“飞行监视页”拆成显式状态，避免再借用退出标志来表示页面切换。
 * 2. 给 cpu2_main 提供一个稳定的状态查询接口，减少对菜单内部细节的耦合。
 */
void menu_enter_monitor_view(void)
{
    g_menu_screen_state = MENU_SCREEN_MONITOR;
}

void menu_enter_main_menu_view(void)
{
    g_menu_screen_state = MENU_SCREEN_MENU;
    g_menu_exit_flag = 0;
}

uint8 menu_is_monitor_view(void)
{
    return (g_menu_screen_state == MENU_SCREEN_MONITOR) ? 1 : 0;
}
