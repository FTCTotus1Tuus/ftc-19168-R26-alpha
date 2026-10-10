package org.firstinspires.ftc.teamcode.v1.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * MoveToCoordinateConfig — dashboard-tunable constants for V1 Move To (96,96) autonomous.
 */
@Config
public final class MoveToCoordinateConfig {
    private MoveToCoordinateConfig() {
    }

    // Absolute field-centric target in inches.
    public static double AUTO_TARGET_X_IN = 96.0;
    public static double AUTO_TARGET_Y_IN = 96.0;

    // Fallback start pose if live localization is unavailable at START.
    public static double AUTO_FALLBACK_START_X_IN = 8.5;
    public static double AUTO_FALLBACK_START_Y_IN = 8.5;
    public static double AUTO_FALLBACK_START_HEADING_RAD = 0.0;

    // Safety watchdogs.
    public static double AUTO_MAX_TIME_S = 8.0;
    public static double AUTO_MAX_TRAVEL_FROM_START_IN = 150.0;

    // Completion criteria.
    public static double AUTO_END_PROGRESS_TOLERANCE_IN = 1.5;
    public static double AUTO_END_SETTLE_TIME_S = 0.10;
}

