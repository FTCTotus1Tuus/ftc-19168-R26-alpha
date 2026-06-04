package org.firstinspires.ftc.teamcode.v1.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * DriveConfig — tunable drive behavior constants for TeleOp.
 * Exposed through FtcDashboard via @Config using uppercase group prefixes.
 */
@Config
public final class DriveConfig {
    private DriveConfig() {
    }

    // Prefixes provide pseudo-groups in FTC Dashboard's flat field list.

    // ENCODER_*: motor encoder conversion values.
    public static double ENCODER_TICKS_PER_ROTATION = 28 * 4;

    // POSE_RESET_*: origin offset from robot center.
    public static double POSE_RESET_ROBOT_CENTER_OFFSET_X = 8.5;
    public static double POSE_RESET_ROBOT_CENTER_OFFSET_Y = 8.25;

    // TELEOP_*: driver control shaping.
    public static double TELEOP_ROTATION_SCALE = 0.5;
    public static double TELEOP_SPEED_SCALE = 1.0;
    public static double TELEOP_SPEED_SCALE_TURN = 0.8;
    public static double TELEOP_INPUT_EXPONENT = 3.0;
    public static double TELEOP_DRIVE_DEADZONE = 0.1;

    /**
     * Selects alliance for field-centric heading offset.
     * true = RED, false = BLUE.
     */
    public static boolean TELEOP_FIELD_CENTRIC_IS_RED_ALLIANCE = true;

    /**
     * Field-centric heading offset applied on RED alliance (radians).
     * Keep at 0.0 if your field frame already aligns with your preferred RED controls.
     */
    public static double TELEOP_FIELD_CENTRIC_RED_OFFSET_RAD = 0.0;

    /**
     * Field-centric heading offset applied on BLUE alliance (radians).
     * Set to Math.PI if BLUE should be flipped 180 degrees from RED controls.
     */
    public static double TELEOP_FIELD_CENTRIC_BLUE_OFFSET_RAD = Math.PI;

    /**
     * Minimum speed multiplier applied when right_trigger is fully depressed.
     * 0.3 = 30% of normal speed at full trigger; blends linearly back to 1.0 at rest.
     * Tune this to whatever feels comfortable for fine scoring maneuvers.
     */
    public static double TELEOP_PRECISION_SCALE = 0.3;

    /** Enables front-wheel RPM limiting to compensate for faster front gearing. */
    public static boolean GEAR_COMP_ENABLE_FRONT_RPM_LIMITING = true;

    /**
     * Max RPM fraction applied to both front motors.
     * 0.75 matches a front drivetrain that is ~4/3 faster than the rear drivetrain.
     */
    public static double GEAR_COMP_FRONT_WHEEL_MAX_RPM_FRACTION = 0.75;
}
