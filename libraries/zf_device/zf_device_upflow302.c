#include "zf_device_upflow302.h"

uint8 upflow302_receiver_data[UP_FLOW_302_DATA_LEN] = {0};
uint8 upflow302_finsh_flag = 0;
uint8 upflow302_state_flag = 0;
uint16 upflow302_response_time = 0;
int16 upflow302_x = 0;
int16 upflow302_y = 0;
upflow302_receive_struct upflow302_receive = {0};

static void upflow302_parse_byte(uint8 receive_data)
{
    static vuint8 length = 0;
    uint8 parity_bit_sum = 0;
    uint8 parity_bit = 0;

    upflow302_receiver_data[length++] = receive_data;

    if((1U == length) && (UP_FLOW_302_FRAME_START != upflow302_receiver_data[0]))
    {
        length = 0;
        return;
    }

    if((2U == length) && (UP_FLOW_302_DEVICE_ID != upflow302_receiver_data[1]))
    {
        length = 0;
        return;
    }

    if(UP_FLOW_302_DATA_LEN <= length)
    {
        parity_bit = upflow302_receiver_data[12];
        upflow302_receiver_data[12] = 0;

        for(uint8 index = 2U; index < 12U; index++)
        {
            parity_bit_sum ^= upflow302_receiver_data[index];
        }

        upflow302_receiver_data[12] = parity_bit;

        if(parity_bit_sum == parity_bit)
        {
            upflow302_finsh_flag = 1;
            upflow302_state_flag = 1;
            upflow302_response_time = 0;
            memcpy((uint8 *)&upflow302_receive, (uint8 *)upflow302_receiver_data, sizeof(upflow302_receive));
            upflow302_x = upflow302_receive.upflow302_x;
            upflow302_y = upflow302_receive.upflow302_y;
        }
        else
        {
            upflow302_finsh_flag = 0;
            upflow302_state_flag = 0;
        }

        length = 0;
    }
}

void upflow302_receive_callback(void)
{
    uint8 receive_data = 0;

    while(uart_query_byte(UP_FLOW_302_UART_INDEX, &receive_data))
    {
        upflow302_parse_byte(receive_data);
    }
}

void upflow302_receive_init(void)
{
    upflow302_finsh_flag = 0;
    upflow302_state_flag = 0;
    upflow302_response_time = 0;
    memset(upflow302_receiver_data, 0, sizeof(upflow302_receiver_data));
    memset((uint8 *)&upflow302_receive, 0, sizeof(upflow302_receive));

    uart_init(UP_FLOW_302_UART_INDEX,
              UP_FLOW_302_BAUDRATE,
              UP_FLOW_302_UART_TX_PIN,
              UP_FLOW_302_UART_RX_PIN);
    uart_rx_interrupt(UP_FLOW_302_UART_INDEX, 1);
}