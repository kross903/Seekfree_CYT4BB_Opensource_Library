#ifndef COMM_MUX_H_
#define COMM_MUX_H_

#include "zf_common_headfile.h"

typedef enum
{
    COMM_MUX_MODE_VOFA = 0,
    COMM_MUX_MODE_ASSISTANT_IMAGE = 1,
} CommMuxMode_t;

void CommMux_Init(void);
void CommMux_SetMode(CommMuxMode_t mode);
CommMuxMode_t CommMux_GetMode(void);
uint8 CommMux_IsVofaMode(void);
uint8 CommMux_IsAssistantImageMode(void);

#endif
