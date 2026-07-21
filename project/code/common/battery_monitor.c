#include "common/battery_monitor.h"
#include "common/flight_safety.h"

#include <math.h>

/* Compatibility mirrors used by existing UI and communication code. */
float voltage = 0.0f;
uint16 adc_value = 0;
volatile float car_bat_voltage = 0.0f;

#define BATTERY_TASK_PERIOD_MS             5U
#define BATTERY_FILTER_ALPHA               0.05f
#define BATTERY_FILTER_READY_SAMPLES       20U
#define BATTERY_WARNING_CONFIRM_SAMPLES    20U
#define BATTERY_REMOTE_TIMEOUT_MS          1000U
#define BATTERY_4S_VALID_MIN_V             10.0f
#define BATTERY_4S_VALID_MAX_V             18.0f
#define BATTERY_6S_VALID_MIN_V             18.0f
#define BATTERY_6S_VALID_MAX_V             27.0f
#define BATTERY_REMOTE_VALID_MIN_V         10.0f
#define BATTERY_REMOTE_VALID_MAX_V         18.0f

static BatteryStatus s_battery_status = {
    .compensation_gain = 0.0f,
    .remote_age_ms = BATTERY_REMOTE_TIMEOUT_MS
};
static uint16 s_filter_sample_count = 0U;
static uint8 s_pack_detect_count = 0U;
static BatteryPackType s_pack_candidate = BATTERY_PACK_UNKNOWN;
static uint8 s_low_sample_count = 0U;
static uint8 s_recover_sample_count = 0U;

static BatteryPackType battery_detect_pack(float voltage_v)
{
    if(isfinite(voltage_v) &&
       voltage_v >= BATTERY_4S_DETECT_MIN_V &&
       voltage_v <= BATTERY_4S_DETECT_MAX_V)
    {
        return BATTERY_PACK_4S;
    }
    if(isfinite(voltage_v) &&
       voltage_v >= BATTERY_6S_DETECT_MIN_V &&
       voltage_v <= BATTERY_6S_DETECT_MAX_V)
    {
        return BATTERY_PACK_6S;
    }
    return BATTERY_PACK_UNKNOWN;
}

static uint8 battery_local_voltage_is_valid(float voltage_v, BatteryPackType pack_type)
{
    if(!isfinite(voltage_v))
    {
        return 0U;
    }
    if(pack_type == BATTERY_PACK_4S)
    {
        return (uint8)(voltage_v >= BATTERY_4S_VALID_MIN_V &&
                       voltage_v <= BATTERY_4S_VALID_MAX_V);
    }
    if(pack_type == BATTERY_PACK_6S)
    {
        return (uint8)(voltage_v >= BATTERY_6S_VALID_MIN_V &&
                       voltage_v <= BATTERY_6S_VALID_MAX_V);
    }
    return 0U;
}

static uint8 battery_remote_voltage_is_valid(float voltage_v)
{
    return (uint8)(isfinite(voltage_v) &&
                   voltage_v >= BATTERY_REMOTE_VALID_MIN_V &&
                   voltage_v <= BATTERY_REMOTE_VALID_MAX_V);
}

static void battery_set_pack_thresholds(BatteryPackType pack_type)
{
    float cell_count = (float)pack_type;

    s_battery_status.low_warn_voltage_v = cell_count * BATTERY_LOW_CELL_WARN_V;
    s_battery_status.low_recover_voltage_v = cell_count * BATTERY_LOW_CELL_RECOVER_V;
}

static float battery_compensation_gain(float voltage_v)
{
    float gain;

    if(!battery_local_voltage_is_valid(voltage_v, s_battery_status.pack_type))
    {
        return 0.0f;
    }

    gain = BATTERY_COMP_REFERENCE_V / voltage_v;
    if(gain > BATTERY_COMP_GAIN_MAX)
    {
        gain = BATTERY_COMP_GAIN_MAX;
    }
    else if(gain < BATTERY_COMP_GAIN_MIN)
    {
        gain = BATTERY_COMP_GAIN_MIN;
    }
    return gain;
}

static void battery_update_warning(float voltage_v)
{
    if(!s_battery_status.local_valid)
    {
        s_low_sample_count = 0U;
        s_recover_sample_count = 0U;
        return;
    }

    if(voltage_v < s_battery_status.low_warn_voltage_v)
    {
        s_recover_sample_count = 0U;
        if(s_low_sample_count < BATTERY_WARNING_CONFIRM_SAMPLES)
        {
            s_low_sample_count++;
        }
        if(s_low_sample_count >= BATTERY_WARNING_CONFIRM_SAMPLES)
        {
            s_battery_status.low_warning = 1U;
        }
    }
    else if(voltage_v >= s_battery_status.low_recover_voltage_v)
    {
        s_low_sample_count = 0U;
        if(s_recover_sample_count < BATTERY_WARNING_CONFIRM_SAMPLES)
        {
            s_recover_sample_count++;
        }
        if(s_recover_sample_count >= BATTERY_WARNING_CONFIRM_SAMPLES)
        {
            s_battery_status.low_warning = 0U;
        }
    }
    else
    {
        s_low_sample_count = 0U;
        s_recover_sample_count = 0U;
    }
}

void BatteryMonitor_Task_5ms(void)
{
    float raw_voltage_v;
    float compensation_voltage_v;
    BatteryPackType detected_pack;
    uint8 sample_valid;

    adc_value = adc_mean_filter_convert(BATTERY_ADC_CHANNEL, 10);
    raw_voltage_v = (float)adc_value * BATTERY_ADC_TO_VOLTAGE_SCALE;

    s_battery_status.adc_raw = adc_value;
    s_battery_status.local_raw_voltage_v = raw_voltage_v;

    if(s_battery_status.pack_type == BATTERY_PACK_UNKNOWN)
    {
        detected_pack = battery_detect_pack(raw_voltage_v);
        if(detected_pack == BATTERY_PACK_UNKNOWN)
        {
            s_pack_candidate = BATTERY_PACK_UNKNOWN;
            s_pack_detect_count = 0U;
            s_filter_sample_count = 0U;
            s_battery_status.local_valid = 0U;
        }
        else
        {
            if(detected_pack != s_pack_candidate)
            {
                s_pack_candidate = detected_pack;
                s_pack_detect_count = 0U;
                s_filter_sample_count = 0U;
            }

            if(s_filter_sample_count == 0U)
            {
                s_battery_status.local_filtered_voltage_v = raw_voltage_v;
            }
            else
            {
                s_battery_status.local_filtered_voltage_v +=
                    BATTERY_FILTER_ALPHA * (raw_voltage_v - s_battery_status.local_filtered_voltage_v);
            }
            if(s_filter_sample_count < BATTERY_FILTER_READY_SAMPLES)
            {
                s_filter_sample_count++;
            }
            if(s_pack_detect_count < BATTERY_FILTER_READY_SAMPLES)
            {
                s_pack_detect_count++;
            }
            if(s_pack_detect_count >= BATTERY_FILTER_READY_SAMPLES)
            {
                s_battery_status.pack_type = s_pack_candidate;
                battery_set_pack_thresholds(s_battery_status.pack_type);
                s_battery_status.local_valid = 1U;
            }
        }
    }
    else
    {
        sample_valid = battery_local_voltage_is_valid(raw_voltage_v, s_battery_status.pack_type);
        if(sample_valid)
        {
            if(s_filter_sample_count == 0U)
            {
                s_battery_status.local_filtered_voltage_v = raw_voltage_v;
            }
            else
            {
                s_battery_status.local_filtered_voltage_v +=
                    BATTERY_FILTER_ALPHA * (raw_voltage_v - s_battery_status.local_filtered_voltage_v);
            }

            if(s_filter_sample_count < BATTERY_FILTER_READY_SAMPLES)
            {
                s_filter_sample_count++;
            }
            s_battery_status.local_valid =
                (uint8)(s_filter_sample_count >= BATTERY_FILTER_READY_SAMPLES);
        }
        else
        {
            s_filter_sample_count = 0U;
            s_battery_status.local_valid = 0U;
        }
    }

    if(s_battery_status.remote_age_ms < BATTERY_REMOTE_TIMEOUT_MS)
    {
        uint32 next_age_ms = (uint32)s_battery_status.remote_age_ms + BATTERY_TASK_PERIOD_MS;
        s_battery_status.remote_age_ms = (uint16)((next_age_ms > BATTERY_REMOTE_TIMEOUT_MS)
                                                   ? BATTERY_REMOTE_TIMEOUT_MS
                                                   : next_age_ms);
    }
    s_battery_status.remote_valid = (uint8)(battery_remote_voltage_is_valid(s_battery_status.remote_voltage_v) &&
                                             (s_battery_status.remote_age_ms < BATTERY_REMOTE_TIMEOUT_MS));

    battery_update_warning(s_battery_status.local_filtered_voltage_v);
    compensation_voltage_v = s_battery_status.local_raw_voltage_v;
    if(s_battery_status.local_filtered_voltage_v > compensation_voltage_v)
    {
        compensation_voltage_v = s_battery_status.local_filtered_voltage_v;
    }
    s_battery_status.compensation_gain = s_battery_status.local_valid
                                           ? battery_compensation_gain(compensation_voltage_v)
                                           : 0.0f;

    voltage = s_battery_status.local_valid ? s_battery_status.local_filtered_voltage_v : 0.0f;
    Flight_Safety_Update_Battery_Warning();
}

void BatteryMonitor_SetRemoteVoltage(float voltage_v)
{
    uint32 irq_state;

    if(!battery_remote_voltage_is_valid(voltage_v))
    {
        return;
    }

    irq_state = __get_PRIMASK();
    __disable_irq();
    car_bat_voltage = voltage_v;
    s_battery_status.remote_voltage_v = voltage_v;
    s_battery_status.remote_age_ms = 0U;
    s_battery_status.remote_valid = 1U;
    if(irq_state == 0U)
    {
        __enable_irq();
    }
}

void BatteryMonitor_GetStatus(BatteryStatus *status)
{
    if(status != NULL)
    {
        uint32 irq_state = __get_PRIMASK();

        __disable_irq();
        *status = s_battery_status;
        if(irq_state == 0U)
        {
            __enable_irq();
        }
    }
}

float BatteryMonitor_GetCompensationGain(void)
{
    return s_battery_status.local_valid ? s_battery_status.compensation_gain : 0.0f;
}
