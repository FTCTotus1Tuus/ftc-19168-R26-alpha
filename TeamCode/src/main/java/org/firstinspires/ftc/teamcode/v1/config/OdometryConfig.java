package org.firstinspires.ftc.teamcode.v1.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * OdometryConfig — Pinpoint/odometry geometry and encoder conversion constants.
 * Keep values dashboard-editable during bring-up, then copy tuned values back here.
 */
@Config
public final class OdometryConfig {
    private OdometryConfig() {
    }

    // Pinpoint pod offsets relative to robot center (inches).
    public static double FORWARD_POD_Y_IN = -2.42;
    public static double STRAFE_POD_X_IN = -2.16;

    // Pinpoint encoder sign controls.
    // If autonomous runs away in "forward" even with correct motor directions,
    // flip FORWARD_ENCODER_REVERSED first.
    public static boolean FORWARD_ENCODER_REVERSED = true;
    public static boolean STRAFE_ENCODER_REVERSED = true;

    // Encoder conversion placeholders (fill with your actual hardware values).
    public static double ODOM_WHEEL_DIAMETER_IN = 1.37795;
    public static double ODOM_TICKS_PER_REV = 8192.0;

    // TODO: add robot-version-specific odometry constants here (e.g., track width, wheelbase).
}

