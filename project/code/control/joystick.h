#ifndef JOYSTICK_H
#define JOYSTICK_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== 摇杆 ADC 参数 ========================== */
#define JOYSTICK_ADC_MIN        0        // ADC 最小值
#define JOYSTICK_ADC_MAX        4000     // ADC 最大值
#define JOYSTICK_ADC_CENTER     2000     // ADC 中值（物理零点）
#define JOYSTICK_DEADZONE_RATIO 0.1f    // 死区比例：中值 ±5%

/* ========================== 函数声明 ============================== */

/**
 * @brief  将摇杆 ADC 原始值归一化到 [-1.0, 1.0]
 *
 * 归一化规则：
 *   - 死区：|adc - center| <= center * deadzone_ratio → 输出 0.0
 *   - 正方向：adc 从 (center + deadzone) 线性映射到 adc_max  → [0.0,  1.0]
 *   - 负方向：adc 从 (center - deadzone) 线性映射到 adc_min  → [0.0, -1.0]
 *
 * @param  raw_adc  ADC 原始采样值（0 ~ 4000）
 * @return float    归一化结果，范围 [-1.0, 1.0]
 */
float joystick_normalize(int raw_adc);

#ifdef __cplusplus
}
#endif

#endif /* JOYSTICK_H */
