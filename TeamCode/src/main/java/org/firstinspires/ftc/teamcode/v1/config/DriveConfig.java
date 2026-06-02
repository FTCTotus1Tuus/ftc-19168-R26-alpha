package org.firstinspires.ftc.teamcode.v1.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * DriveConfig — tunable drive behavior constants for TeleOp.
 * Exposed through FtcDashboard via @Config.
 */
@Config
public final class DriveConfig {
    private DriveConfig() {
    }

    // Encoder constants for goBILDA 5203 in 4x mode.
    public static double TICKS_PER_ROTATION = 28 * 4;

    // Human-player reset origin offset from robot center.
    public static double ROBOT_CENTER_OFFSET_X = 8.5;
    public static double ROBOT_CENTER_OFFSET_Y = 8.25;

    // TeleOp drive shaping.
    public static double ROTATION_SCALE = 0.5;
    public static double SPEED_SCALE = 1.0;
    public static double SPEED_SCALE_TURN = 0.8;
    public static double INPUT_EXPONENT = 3.0;
    public static double DRIVE_DEADZONE = 0.1;

    /**
     * Minimum speed multiplier applied when right_trigger is fully depressed.
     * 0.3 = 30% of normal speed at full trigger; blends linearly back to 1.0 at rest.
     * Tune this to whatever feels comfortable for fine scoring maneuvers.
     */
    public static double PRECISION_SCALE = 0.3;

    /** Enables front-wheel RPM limiting to compensate for faster front gearing. */
    public static boolean ENABLE_FRONT_GEAR_COMPENSATION = true;

    /**
     * Max RPM fraction applied to both front motors.
     * 0.75 matches a front drivetrain that is ~4/3 faster than the rear drivetrain.
     */
    public static double FRONT_WHEEL_MAX_RPM_FRACTION = 0.75;
}

