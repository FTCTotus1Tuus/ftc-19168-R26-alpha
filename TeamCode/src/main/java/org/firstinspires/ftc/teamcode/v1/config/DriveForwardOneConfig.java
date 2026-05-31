package org.firstinspires.ftc.teamcode.v1.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * DriveForwardOneConfig — route-specific constants for DriveForwardOneAuto.
 * Keep this class focused on a single OpMode to avoid bloating shared AutoConfig.
 */
@Config
public final class DriveForwardOneConfig {
    private DriveForwardOneConfig() {
    }

    // Start pose (inches, radians).
    public static double START_X_IN = AutoConfig.START_X_IN;
    public static double START_Y_IN = AutoConfig.START_Y_IN;
    public static double START_HEADING_RAD = AutoConfig.START_HEADING_RAD;

    // End pose defaults: one tile forward from start while holding heading.
    public static double END_X_IN = START_X_IN;
    public static double END_Y_IN = START_Y_IN + 24.0;
    public static double END_HEADING_RAD = START_HEADING_RAD;

    // TODO: add route-specific toggles here (e.g., holdEndPose, reversePath, reducedSpeedMode).
}

