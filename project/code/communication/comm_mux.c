#include "communication/comm_mux.h"

static volatile CommMuxMode_t s_comm_mux_mode = COMM_MUX_MODE_VOFA;

void CommMux_Init(void)
{
    s_comm_mux_mode = COMM_MUX_MODE_VOFA;
}

void CommMux_SetMode(CommMuxMode_t mode)
{
    s_comm_mux_mode = mode;
}

CommMuxMode_t CommMux_GetMode(void)
{
    return s_comm_mux_mode;
}

uint8 CommMux_IsVofaMode(void)
{
    return (s_comm_mux_mode == COMM_MUX_MODE_VOFA) ? 1U : 0U;
}

uint8 CommMux_IsAssistantImageMode(void)
{
    return (s_comm_mux_mode == COMM_MUX_MODE_ASSISTANT_IMAGE) ? 1U : 0U;
}
