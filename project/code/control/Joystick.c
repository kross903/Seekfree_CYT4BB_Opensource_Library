#include "joystick.h"

/* ===================== 内部常量（由宏计算得到） ===================== */

/* 死区半宽 = 中值 * 死区比例 = 1950 * 0.05 = 97.5 */
static const float DEADZONE_HALF = JOYSTICK_ADC_CENTER * JOYSTICK_DEADZONE_RATIO;

/* 可用行程（死区边界到端点） */
static const float RANGE_POS     = JOYSTICK_ADC_MAX - (JOYSTICK_ADC_CENTER + JOYSTICK_ADC_CENTER * JOYSTICK_DEADZONE_RATIO);
static const float RANGE_NEG     = (JOYSTICK_ADC_CENTER - JOYSTICK_ADC_CENTER * JOYSTICK_DEADZONE_RATIO) - JOYSTICK_ADC_MIN;

/* ========================== 归一化函数 ============================ */

float joystick_normalize(int raw_adc)
{

    
    float adc = raw_adc;
    float delta = adc - JOYSTICK_ADC_CENTER;

    /* 1. 死区判断 */
    if (delta > -DEADZONE_HALF && delta < DEADZONE_HALF)
    {
        return 0.0f;
    }

    /* 2. 正方向：adc > center + deadzone  →  [0.0, 1.0] */
    if (delta >= DEADZONE_HALF)
    {
        float output = (delta - DEADZONE_HALF) / RANGE_POS;
        if (output > 1.0f) output = 1.0f;
        return output;
    }

    /* 3. 负方向：adc < center - deadzone  →  [-1.0, 0.0] */
    float output = (delta + DEADZONE_HALF) / RANGE_NEG;
    if (output < -1.0f) output = -1.0f;
    return output;
}
