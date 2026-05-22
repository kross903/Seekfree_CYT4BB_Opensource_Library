#include "communication/comm_mux.h"

static volatile CommMuxMode_t s_comm_mux_mode = COMM_MUX_MODE_VOFA;

void CommMux_Init(void)
{
    s_comm_mux_mode = COMM_MUX_MODE_VOFA;
}

void CommMux_SwitchInit(void)
{
    gpio_init(COMM_MUX_SWITCH_0_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(COMM_MUX_SWITCH_1_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
}

void CommMux_SetMode(CommMuxMode_t mode)
{
    s_comm_mux_mode = mode;
}

CommMuxMode_t CommMux_GetMode(void)
{
    return s_comm_mux_mode;
}

uint8 CommMux_UpdateFromSwitch(void)
{
    CommMuxMode_t next_mode = COMM_MUX_MODE_VOFA;
    uint8 switch_0 = gpio_get_level(COMM_MUX_SWITCH_0_PIN);
    uint8 switch_1 = gpio_get_level(COMM_MUX_SWITCH_1_PIN);

    if((switch_0 == GPIO_LOW) && (switch_1 == GPIO_LOW))
    {
        next_mode = COMM_MUX_MODE_ASSISTANT_IMAGE;
    }

    if(s_comm_mux_mode != next_mode)
    {
        s_comm_mux_mode = next_mode;
        return 1U;
    }

    return 0U;
}

uint8 CommMux_IsVofaMode(void)
{
    return (s_comm_mux_mode == COMM_MUX_MODE_VOFA) ? 1U : 0U;
}

uint8 CommMux_IsAssistantImageMode(void)
{
    return (s_comm_mux_mode == COMM_MUX_MODE_ASSISTANT_IMAGE) ? 1U : 0U;
}
