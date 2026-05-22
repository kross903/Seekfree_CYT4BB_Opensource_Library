#ifndef COMM_MUX_H_
#define COMM_MUX_H_

#include "zf_common_headfile.h"

#define COMM_MUX_SWITCH_0_PIN       (P09_0)
#define COMM_MUX_SWITCH_1_PIN       (P09_1)

typedef enum
{
    COMM_MUX_MODE_VOFA = 0,
    COMM_MUX_MODE_ASSISTANT_IMAGE = 1,
} CommMuxMode_t;

void CommMux_Init(void);
void CommMux_SwitchInit(void);
void CommMux_SetMode(CommMuxMode_t mode);
CommMuxMode_t CommMux_GetMode(void);
uint8 CommMux_UpdateFromSwitch(void);
uint8 CommMux_IsVofaMode(void);
uint8 CommMux_IsAssistantImageMode(void);

#endif
