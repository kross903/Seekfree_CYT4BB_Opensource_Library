#include "control/dual_tof_manager.h"
#include "common/project_headfile.h"

#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_soft_iic.h"
#include "zf_device_config.h"
#include "zf_device_dl1b.h"

#include <string.h>

extern uint8 dl1b_init_flag;

#define DUAL_TOF2_SCL_PIN              (P06_0)
#define DUAL_TOF2_SDA_PIN              (P06_1)
#define DUAL_TOF2_XS_PIN               (P06_2)
#define DUAL_TOF2_DEV_ADDR             (0x52 >> 1)
#define DUAL_TOF2_SOFT_IIC_DELAY       (100U)
#define DUAL_TOF2_TIMEOUT_COUNT        (1000U)

#define DUAL_TOF_VALID_MIN_MM          (20U)
#define DUAL_TOF_VALID_MAX_MM          (2000U)
#define DUAL_TOF_CACHE_TIMEOUT_MS      (100U)
#define DUAL_TOF_POLL_PERIOD_MS        (10U)

#define DL1B_GPIO_TIO_HV_STATUS        (0x0031U)
#define DL1B_SYSTEM_INTERRUPT_CLEAR    (0x0086U)
#define DL1B_RESULT_RANGE_STATUS       (0x0089U)
#define DL1B_RESULT_RANGE_MM           (0x0096U)
#define DL1B_FIRMWARE_SYSTEM_STATUS   (0x00E5U)
#define DL1B_I2C_DEVICE_ADDRESS        (0x0001U)

static soft_iic_info_struct s_tof2_iic;
static uint8 s_tof2_initialized = 0U;
static uint16 s_tof1_raw_mm = 8192U;
static uint16 s_tof2_distance_mm = 8192U;
static uint32 s_tof1_invalid_count = 0U;
static uint32 s_tof2_invalid_count = 0U;
static uint32 s_tof2_bus_error_count = 0U;

static uint16 s_tof1_last_valid_mm = 0U;
static uint16 s_tof2_last_valid_mm = 0U;
static uint16 s_tof1_age_ms = 0xFFFFU;
static uint16 s_tof2_age_ms = 0xFFFFU;
static uint8 s_tof1_have_valid = 0U;
static uint8 s_tof2_have_valid = 0U;

static void dual_tof_increment_age(uint16 *age_ms)
{
    if(*age_ms <= (uint16)(0xFFFFU - DUAL_TOF_POLL_PERIOD_MS))
    {
        *age_ms = (uint16)(*age_ms + DUAL_TOF_POLL_PERIOD_MS);
    }
    else
    {
        *age_ms = 0xFFFFU;
    }
}

static uint8 dual_tof_range_valid(uint16 distance_mm)
{
    return (uint8)((distance_mm >= DUAL_TOF_VALID_MIN_MM) &&
                   (distance_mm <= DUAL_TOF_VALID_MAX_MM));
}

static void dual_tof2_transfer(const uint8 *write_data, uint32 write_len,
                               uint8 *read_data, uint32 read_len)
{
    soft_iic_transfer_8bit_array(&s_tof2_iic, write_data, write_len, read_data, read_len);
}

static uint8 dual_tof2_read_distance(void)
{
    uint8 data_buffer[3] = {0U};
    uint16 distance_mm;

    if(!s_tof2_initialized)
    {
        return 0U;
    }

    data_buffer[0] = (uint8)(DL1B_GPIO_TIO_HV_STATUS >> 8);
    data_buffer[1] = (uint8)(DL1B_GPIO_TIO_HV_STATUS & 0xFFU);
    dual_tof2_transfer(data_buffer, 2U, &data_buffer[2], 1U);
    if(!data_buffer[2])
    {
        return 0U;
    }

    data_buffer[0] = (uint8)(DL1B_SYSTEM_INTERRUPT_CLEAR >> 8);
    data_buffer[1] = (uint8)(DL1B_SYSTEM_INTERRUPT_CLEAR & 0xFFU);
    data_buffer[2] = 0x01U;
    dual_tof2_transfer(data_buffer, 3U, data_buffer, 0U);

    data_buffer[0] = (uint8)(DL1B_RESULT_RANGE_STATUS >> 8);
    data_buffer[1] = (uint8)(DL1B_RESULT_RANGE_STATUS & 0xFFU);
    dual_tof2_transfer(data_buffer, 2U, &data_buffer[2], 1U);
    if(data_buffer[2] != 0x89U)
    {
        s_tof2_distance_mm = 8192U;
        s_tof2_invalid_count++;
        return 0U;
    }

    data_buffer[0] = (uint8)(DL1B_RESULT_RANGE_MM >> 8);
    data_buffer[1] = (uint8)(DL1B_RESULT_RANGE_MM & 0xFFU);
    dual_tof2_transfer(data_buffer, 2U, data_buffer, 2U);
    distance_mm = (uint16)(((uint16)data_buffer[0] << 8) | data_buffer[1]);
    if(distance_mm > 4000U)
    {
        s_tof2_distance_mm = 8192U;
        s_tof2_invalid_count++;
        return 0U;
    }

    s_tof2_distance_mm = distance_mm;
    return 1U;
}

static uint8 dual_tof2_init_sensor(void)
{
    uint8 return_state = 0U;
    uint8 data_buffer[2U + sizeof(dl1b_config_file)];
    uint16 timeout_count = 0U;

    soft_iic_init(&s_tof2_iic, DUAL_TOF2_DEV_ADDR, DUAL_TOF2_SOFT_IIC_DELAY,
                  DUAL_TOF2_SCL_PIN, DUAL_TOF2_SDA_PIN);
    gpio_init(DUAL_TOF2_XS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    system_delay_ms(50U);
    gpio_low(DUAL_TOF2_XS_PIN);
    system_delay_ms(10U);
    gpio_high(DUAL_TOF2_XS_PIN);
    system_delay_ms(50U);

    data_buffer[0] = (uint8)(DL1B_FIRMWARE_SYSTEM_STATUS >> 8);
    data_buffer[1] = (uint8)(DL1B_FIRMWARE_SYSTEM_STATUS & 0xFFU);
    dual_tof2_transfer(data_buffer, 2U, &data_buffer[2], 1U);
    if((data_buffer[2] & 0x01U) != 0x01U)
    {
        s_tof2_bus_error_count++;
        return 1U;
    }

    data_buffer[0] = (uint8)(DL1B_I2C_DEVICE_ADDRESS >> 8);
    data_buffer[1] = (uint8)(DL1B_I2C_DEVICE_ADDRESS & 0xFFU);
    memcpy(&data_buffer[2], dl1b_config_file, sizeof(dl1b_config_file));

    /* Keep the same LONG-range and timing configuration as the existing ToF. */
    data_buffer[1U + 0x004BU] = 0x0AU;
    data_buffer[1U + 0x005EU] = 0x00U;
    data_buffer[1U + 0x005FU] = 0x60U;
    data_buffer[1U + 0x0060U] = 0x0FU;
    data_buffer[1U + 0x0061U] = 0x00U;
    data_buffer[1U + 0x0062U] = 0x6EU;
    data_buffer[1U + 0x0063U] = 0x0DU;
    data_buffer[1U + 0x0069U] = 0xB8U;
    data_buffer[1U + 0x0078U] = 0x0FU;
    data_buffer[1U + 0x0079U] = 0x0DU;
    data_buffer[1U + 0x007AU] = 0x0EU;
    data_buffer[1U + 0x007BU] = 0x0EU;
    dual_tof2_transfer(data_buffer, 2U + sizeof(dl1b_config_file), data_buffer, 0U);

    while(1)
    {
        data_buffer[0] = (uint8)(DL1B_GPIO_TIO_HV_STATUS >> 8);
        data_buffer[1] = (uint8)(DL1B_GPIO_TIO_HV_STATUS & 0xFFU);
        dual_tof2_transfer(data_buffer, 2U, &data_buffer[2], 1U);
        if((data_buffer[2] & 0x01U) == 0U)
        {
            break;
        }
        if(timeout_count++ >= DUAL_TOF2_TIMEOUT_COUNT)
        {
            s_tof2_bus_error_count++;
            return_state = 1U;
            break;
        }
        system_delay_ms(1U);
    }

    if(return_state == 0U)
    {
        s_tof2_initialized = 1U;
    }
    return return_state;
}

uint8 DualTof_Init(void)
{
    s_tof2_initialized = 0U;
    s_tof1_raw_mm = 8192U;
    s_tof2_distance_mm = 8192U;
    s_tof1_last_valid_mm = 0U;
    s_tof2_last_valid_mm = 0U;
    s_tof1_age_ms = 0xFFFFU;
    s_tof2_age_ms = 0xFFFFU;
    s_tof1_have_valid = 0U;
    s_tof2_have_valid = 0U;
    s_tof1_invalid_count = 0U;
    s_tof2_invalid_count = 0U;
    s_tof2_bus_error_count = 0U;
    return dual_tof2_init_sensor();
}

void DualTof_Poll(DualTofSample_t *sample)
{
    uint8 tof1_new;
    uint8 tof2_new;
    uint8 tof1_current_valid;
    uint8 tof2_current_valid;

    if(sample == NULL)
    {
        return;
    }
    memset(sample, 0, sizeof(*sample));

    dual_tof_increment_age(&s_tof1_age_ms);
    dual_tof_increment_age(&s_tof2_age_ms);

    dl1b_get_distance();
    tof1_new = dl1b_finsh_flag;
    tof2_new = dual_tof2_read_distance();

    if(tof1_new)
    {
        s_tof1_raw_mm = dl1b_distance_mm;
    }

    tof1_current_valid = (uint8)(tof1_new && dual_tof_range_valid(dl1b_distance_mm));
    tof2_current_valid = (uint8)(tof2_new && dual_tof_range_valid(s_tof2_distance_mm));

    if(tof1_new && !tof1_current_valid)
    {
        s_tof1_invalid_count++;
    }
    if(tof2_new && !tof2_current_valid)
    {
        s_tof2_invalid_count++;
    }

    if(tof1_current_valid)
    {
        s_tof1_last_valid_mm = dl1b_distance_mm;
        s_tof1_age_ms = 0U;
        s_tof1_have_valid = 1U;
    }
    if(tof2_current_valid)
    {
        s_tof2_last_valid_mm = s_tof2_distance_mm;
        s_tof2_age_ms = 0U;
        s_tof2_have_valid = 1U;
    }
    if(s_tof1_age_ms >= DUAL_TOF_CACHE_TIMEOUT_MS)
    {
        s_tof1_have_valid = 0U;
    }
    if(s_tof2_age_ms >= DUAL_TOF_CACHE_TIMEOUT_MS)
    {
        s_tof2_have_valid = 0U;
    }

    sample->tof1_raw_mm = s_tof1_raw_mm;
    sample->tof2_raw_mm = s_tof2_distance_mm;
    sample->tof1_age_ms = s_tof1_age_ms;
    sample->tof2_age_ms = s_tof2_age_ms;
    sample->tof1_new_sample = tof1_new;
    sample->tof2_new_sample = tof2_new;
    sample->tof1_valid = s_tof1_have_valid;
    sample->tof2_valid = s_tof2_have_valid;
    sample->tof1_initialized = dl1b_init_flag;
    sample->tof2_initialized = s_tof2_initialized;
    sample->tof1_invalid_count = s_tof1_invalid_count;
    sample->tof2_invalid_count = s_tof2_invalid_count;
    sample->tof2_bus_error_count = s_tof2_bus_error_count;

    sample->selected_new_sample = (uint8)(tof1_current_valid || tof2_current_valid);
    if(s_tof1_have_valid && s_tof2_have_valid)
    {
        sample->disagreement_mm = (s_tof1_last_valid_mm >= s_tof2_last_valid_mm) ?
                                  (uint16)(s_tof1_last_valid_mm - s_tof2_last_valid_mm) :
                                  (uint16)(s_tof2_last_valid_mm - s_tof1_last_valid_mm);
        if(s_tof1_last_valid_mm > s_tof2_last_valid_mm)
        {
            sample->selected_mm = s_tof1_last_valid_mm;
            sample->source = DUAL_TOF_SOURCE_TOF1;
        }
        else if(s_tof2_last_valid_mm > s_tof1_last_valid_mm)
        {
            sample->selected_mm = s_tof2_last_valid_mm;
            sample->source = DUAL_TOF_SOURCE_TOF2;
        }
        else
        {
            sample->selected_mm = s_tof1_last_valid_mm;
            sample->source = DUAL_TOF_SOURCE_EQUAL;
        }
        sample->selected_valid = 1U;
    }
    else if(s_tof1_have_valid)
    {
        sample->selected_mm = s_tof1_last_valid_mm;
        sample->source = DUAL_TOF_SOURCE_TOF1;
        sample->selected_valid = 1U;
    }
    else if(s_tof2_have_valid)
    {
        sample->selected_mm = s_tof2_last_valid_mm;
        sample->source = DUAL_TOF_SOURCE_TOF2;
        sample->selected_valid = 1U;
    }
}
