#include "beacon_adaptive_threshold.h"
#include "vision/camera_calibration_params.h"

typedef struct
{
    float height_area_scale;
    float tilt_ratio;
    float tilt_area_scale;
} BeaconAdaptiveFrame_t;

static BeaconAdaptiveFrame_t s_beacon_adaptive_frame;

static float BeaconAdaptive_Clamp(float value, float minimum, float maximum)
{
    if(value < minimum) return minimum;
    if(value > maximum) return maximum;
    return value;
}

static float BeaconAdaptive_FastMagnitude(float x, float y)
{
    float maximum;
    float minimum;

    if(x < 0.0f) x = -x;
    if(y < 0.0f) y = -y;
    maximum = (x > y) ? x : y;
    minimum = (x > y) ? y : x;
    return maximum + 0.375f * minimum;
}

void BeaconAdaptiveThreshold_Init(void)
{
    BeaconAdaptiveThreshold_UpdateFrame(0.0f, 0.0f,
                                        BEACON_ADAPTIVE_REFERENCE_HEIGHT_CM);
}

void BeaconAdaptiveThreshold_UpdateFrame(float roll_deg,
                                         float pitch_deg,
                                         float height_cm)
{
    float height_ratio;
    float tilt_deg;

    height_cm = BeaconAdaptive_Clamp(height_cm,
                                     BEACON_ADAPTIVE_HEIGHT_MIN_CM,
                                     BEACON_ADAPTIVE_HEIGHT_MAX_CM);
    height_ratio = BEACON_ADAPTIVE_REFERENCE_HEIGHT_CM / height_cm;
    s_beacon_adaptive_frame.height_area_scale = height_ratio * height_ratio;

    tilt_deg = BeaconAdaptive_FastMagnitude(roll_deg, pitch_deg);
    s_beacon_adaptive_frame.tilt_ratio = BeaconAdaptive_Clamp(
        tilt_deg / BEACON_ADAPTIVE_TILT_FULL_RELAX_DEG, 0.0f, 1.0f);
    s_beacon_adaptive_frame.tilt_area_scale = 1.0f -
        BEACON_ADAPTIVE_AREA_TILT_LOSS * s_beacon_adaptive_frame.tilt_ratio;
}

void BeaconAdaptiveThreshold_Evaluate(float center_col,
                                      float center_row,
                                      float area_px,
                                      const BeaconAdaptiveBaseGate_t *base,
                                      BeaconAdaptiveGate_t *gate)
{
    float radius;
    float dx;
    float dy;
    float radial_area_scale;
    float relaxation;

    if(base == 0 || gate == 0) return;

    radius = CAMERA_CALIBRATION_CX;
    if(((float)CAMERA_CALIBRATION_IMAGE_WIDTH - 1.0f -
        CAMERA_CALIBRATION_CX) > radius)
    {
        radius = (float)CAMERA_CALIBRATION_IMAGE_WIDTH - 1.0f -
                 CAMERA_CALIBRATION_CX;
    }
    if(radius < 1.0f) radius = 1.0f;
    dx = (center_col - CAMERA_CALIBRATION_CX) / radius;
    dy = (center_row - CAMERA_CALIBRATION_CY) / radius;
    gate->normalized_radius_sq = BeaconAdaptive_Clamp(dx * dx + dy * dy,
                                                       0.0f, 1.0f);
    gate->tilt_ratio = s_beacon_adaptive_frame.tilt_ratio;

    radial_area_scale = 1.0f - BEACON_ADAPTIVE_AREA_RADIAL_LOSS *
                               gate->normalized_radius_sq;
    gate->area_scale = s_beacon_adaptive_frame.height_area_scale *
                       s_beacon_adaptive_frame.tilt_area_scale *
                       radial_area_scale;
    gate->area_scale = BeaconAdaptive_Clamp(gate->area_scale,
                                             BEACON_ADAPTIVE_AREA_SCALE_MIN,
                                             BEACON_ADAPTIVE_AREA_SCALE_MAX);
    gate->normalized_area_px = area_px / gate->area_scale;

    relaxation = BEACON_ADAPTIVE_SHAPE_RADIAL_RELAXATION *
                     gate->normalized_radius_sq +
                 BEACON_ADAPTIVE_SHAPE_TILT_RELAXATION * gate->tilt_ratio;
    gate->relaxation = BeaconAdaptive_Clamp(relaxation, 0.0f,
                                             BEACON_ADAPTIVE_MAX_RELAXATION);

    gate->min_area_px = base->min_area_px * (1.0f - 0.20f * gate->relaxation);
    gate->ideal_area_px = base->ideal_area_px;
    gate->max_area_px = base->max_area_px * (1.0f + 0.25f * gate->relaxation);
    gate->min_aspect = base->min_aspect * (1.0f - 0.45f * gate->relaxation);
    gate->max_aspect = base->max_aspect * (1.0f + 0.50f * gate->relaxation);
    gate->min_occupancy = base->min_occupancy *
                          (1.0f - 0.40f * gate->relaxation);
    gate->min_circularity = base->min_circularity *
                            (1.0f - 0.65f * gate->relaxation);
    gate->box_minimum_scale = 1.0f - 0.25f * gate->relaxation;
}
