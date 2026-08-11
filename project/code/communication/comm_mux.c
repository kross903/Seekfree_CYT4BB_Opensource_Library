#include "communication/comm_mux.h"

#define COMM_MUX_DEBOUNCE_US        (50000U)
#define COMM_MUX_DWT_LAR_UNLOCK     (0xC5ACCE55UL)

static volatile CommMuxMode_t s_comm_mux_mode = COMM_MUX_MODE_VOFA;
static volatile CommMuxMode_t s_comm_mux_requested_mode = COMM_MUX_MODE_VOFA;
static CommMuxMode_t s_comm_mux_candidate_mode = COMM_MUX_MODE_VOFA;
static uint32 s_comm_mux_candidate_since_cycle = 0U;
static uint8 s_comm_mux_candidate_timing_valid = 0U;
static uint8 s_comm_mux_mode_locked = 0U;

static uint8 CommMux_DwtInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    if((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0U)
    {
        return 0U;
    }

    DWT->LAR = COMM_MUX_DWT_LAR_UNLOCK;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    return 1U;
}

static CommMuxMode_t CommMux_ReadSwitchMode(void)
{
    uint8 switch_0 = gpio_get_level(COMM_MUX_SWITCH_0_PIN);
    uint8 switch_1 = gpio_get_level(COMM_MUX_SWITCH_1_PIN);

    if((switch_0 == GPIO_LOW) && (switch_1 == GPIO_LOW))
    {
        return COMM_MUX_MODE_ASSISTANT_IMAGE;
    }
    return COMM_MUX_MODE_VOFA;
}

static uint8 CommMux_IsValidMode(CommMuxMode_t mode)
{
    return (uint8)((mode == COMM_MUX_MODE_VOFA) ||
                   (mode == COMM_MUX_MODE_ASSISTANT_IMAGE) ||
                   (mode == COMM_MUX_MODE_CAMERA_BOARD));
}

void CommMux_Init(void)
{
    s_comm_mux_mode = COMM_MUX_MODE_VOFA;
    s_comm_mux_requested_mode = COMM_MUX_MODE_VOFA;
    s_comm_mux_candidate_mode = COMM_MUX_MODE_VOFA;
    s_comm_mux_candidate_since_cycle = 0U;
    s_comm_mux_candidate_timing_valid = 0U;
    s_comm_mux_mode_locked = 0U;
    (void)CommMux_DwtInit();
}

void CommMux_SwitchInit(void)
{
    gpio_init(COMM_MUX_SWITCH_0_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(COMM_MUX_SWITCH_1_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);

    /* 启动阶段直接采用稳定的物理拨码，避免向错误协议端发送启动文本。 */
    s_comm_mux_mode = CommMux_ReadSwitchMode();
    s_comm_mux_requested_mode = s_comm_mux_mode;
    s_comm_mux_candidate_mode = s_comm_mux_mode;
    s_comm_mux_candidate_timing_valid = 0U;
}

void CommMux_SetMode(CommMuxMode_t mode)
{
    if(!CommMux_IsValidMode(mode))
    {
        return;
    }
    s_comm_mux_mode = mode;
    s_comm_mux_requested_mode = mode;
    s_comm_mux_candidate_mode = mode;
    s_comm_mux_candidate_timing_valid = 0U;
}

CommMuxMode_t CommMux_GetMode(void)
{
    return s_comm_mux_mode;
}

CommMuxMode_t CommMux_GetRequestedMode(void)
{
    return s_comm_mux_requested_mode;
}

uint8 CommMux_UpdateFromSwitch(void)
{
    return CommMux_Service(0U);
}

uint8 CommMux_Service(uint8 armed_state)
{
    CommMuxMode_t sampled_mode = CommMux_ReadSwitchMode();
    uint32 now_cycle;
    uint32 stable_us;

    s_comm_mux_mode_locked = armed_state ? 1U : 0U;
    if(sampled_mode != s_comm_mux_candidate_mode)
    {
        s_comm_mux_candidate_mode = sampled_mode;
        s_comm_mux_candidate_timing_valid = 0U;
    }

    if(!CommMux_DwtInit() || (SystemCoreClock == 0U))
    {
        /* 无高精度时钟时保持当前模式，避免无消抖切换。 */
        return 0U;
    }

    now_cycle = DWT->CYCCNT;
    if(!s_comm_mux_candidate_timing_valid)
    {
        s_comm_mux_candidate_since_cycle = now_cycle;
        s_comm_mux_candidate_timing_valid = 1U;
        return 0U;
    }

    stable_us = (uint32)(((uint64)(now_cycle - s_comm_mux_candidate_since_cycle) *
                           1000000ULL) / (uint64)SystemCoreClock);
    if(stable_us < COMM_MUX_DEBOUNCE_US)
    {
        return 0U;
    }

    s_comm_mux_requested_mode = s_comm_mux_candidate_mode;
    if(s_comm_mux_mode_locked ||
       (s_comm_mux_mode == s_comm_mux_requested_mode))
    {
        return 0U;
    }

    s_comm_mux_mode = s_comm_mux_requested_mode;
    return 1U;
}

uint8 CommMux_IsModeLocked(void)
{
    return s_comm_mux_mode_locked;
}

uint8 CommMux_IsVofaMode(void)
{
    return (s_comm_mux_mode == COMM_MUX_MODE_VOFA) ? 1U : 0U;
}

uint8 CommMux_IsAssistantImageMode(void)
{
    return (s_comm_mux_mode == COMM_MUX_MODE_ASSISTANT_IMAGE) ? 1U : 0U;
}

uint8 CommMux_IsCameraBoardMode(void)
{
    return (s_comm_mux_mode == COMM_MUX_MODE_CAMERA_BOARD) ? 1U : 0U;
}
