#include "zf_common_headfile.h"
#include "../code/PIT_Demo.h"

void pit0_ch0_isr(void)
{
    pit_isr_flag_clear(PIT_CH0);
    PIT_KeyVoltage_Task_5ms();
    PIT_Adc_Task_5ms();
}

void pit0_ch1_isr(void)
{
    pit_isr_flag_clear(PIT_CH1);
    PIT_Control_Task_250us();
}

void pit0_ch2_isr(void)
{
    pit_isr_flag_clear(PIT_CH2);
    PIT_Motor_Task_500us();
}

void uart0_isr(void)
{
    if(uart_isr_mask(UART_0))
    {
#if DEBUG_UART_USE_INTERRUPT
        debug_interrupr_handler();
#endif
    }
}
