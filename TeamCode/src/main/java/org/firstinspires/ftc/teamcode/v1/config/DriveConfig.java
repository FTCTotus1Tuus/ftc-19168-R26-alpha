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

    /** Driver turn sensitivity multiplier applied in DriveSubsystem. */
    public static double TURN_SCALE = 0.45;

    /** Enables front-wheel RPM limiting to compensate for faster front gearing. */
    public static boolean ENABLE_FRONT_GEAR_COMPENSATION = true;

    /**
     * Max RPM fraction applied to both front motors.
     * 0.75 matches a front drivetrain that is ~4/3 faster than the rear drivetrain.
     */
    public static double FRONT_WHEEL_MAX_RPM_FRACTION = 0.75;
}

