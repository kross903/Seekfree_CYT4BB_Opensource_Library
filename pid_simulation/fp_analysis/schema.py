from __future__ import annotations

FP_COLUMNS = {
    "I0": "x_pos_fb_cm",
    "I1": "x_v_target_cms",
    "I2": "x_v_fb_cms",
    "I3": "pitch_target_deg",
    "I4": "pitch_current_deg",
    "I5": "pitch_rate_target_dps",
    "I6": "pitch_rate_current_dps",
    "I7": "y_pos_fb_cm",
    "I8": "y_v_target_cms",
    "I9": "y_v_fb_cms",
    "I10": "roll_target_deg",
    "I11": "roll_current_deg",
    "I12": "roll_rate_target_dps",
    "I13": "roll_rate_current_dps",
    "I14": "position_loop_active",
    "I15": "reserved",
}

NUMERIC_EXCLUDE = ("sample", "time_s")

AXIS_CHAINS = {
    "x": {
        "title": "X/Pitch",
        "filename_prefix": "FP_x",
        "series": [
            ("position", "x_pos_fb_cm", None),
            ("velocity", "x_v_target_cms", "x_v_fb_cms"),
            ("angle", "pitch_target_deg", "pitch_current_deg"),
            ("rate", "pitch_rate_target_dps", "pitch_rate_current_dps"),
        ],
        "cross_axis_targets": ("roll_target_deg", "roll_rate_target_dps"),
        "cross_axis_feedbacks": ("roll_current_deg", "roll_rate_current_dps"),
    },
    "y": {
        "title": "Y/Roll",
        "filename_prefix": "FP_y",
        "series": [
            ("position", "y_pos_fb_cm", None),
            ("velocity", "y_v_target_cms", "y_v_fb_cms"),
            ("angle", "roll_target_deg", "roll_current_deg"),
            ("rate", "roll_rate_target_dps", "roll_rate_current_dps"),
        ],
        "cross_axis_targets": ("pitch_target_deg", "pitch_rate_target_dps"),
        "cross_axis_feedbacks": ("pitch_current_deg", "pitch_rate_current_dps"),
    },
}

DEFAULT_SATURATION_HINTS = {
    "pitch_current_deg": (-90.0, 90.0),
    "roll_current_deg": (-90.0, 90.0),
}

