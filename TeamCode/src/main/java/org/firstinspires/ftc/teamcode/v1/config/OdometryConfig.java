package org.firstinspires.ftc.teamcode.v1.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * OdometryConfig — Pinpoint/odometry geometry and encoder conversion constants.
 * Keep values dashboard-editable during bring-up, then copy tuned values back here.
 * Uses uppercase group prefixes so related fields stay grouped in Dashboard.
 */
@Config
public final class OdometryConfig {
    private OdometryConfig() {
    }

    // Prefixes provide pseudo-groups in FTC Dashboard's flat field list.

    // PINPOINT_*: pod geometry and encoder sign controls.
    public static double PINPOINT_FORWARD_POD_Y_IN = -2.42;
    public static double PINPOINT_STRAFE_POD_X_IN = -2.16;

    // Pinpoint encoder sign controls.
    // If autonomous runs away in "forward" even with correct motor directions,
    // flip PINPOINT_FORWARD_ENCODER_REVERSED first.
    public static boolean PINPOINT_FORWARD_ENCODER_REVERSED = true;
    public static boolean PINPOINT_STRAFE_ENCODER_REVERSED = true;

    // ENCODER_*: raw odometry wheel conversion placeholders.
    public static double ENCODER_ODOM_WHEEL_DIAMETER_IN = 1.37795;
    public static double ENCODER_ODOM_TICKS_PER_REV = 8192.0;

    // TODO: add robot-version-specific odometry constants here (e.g., track width, wheelbase).
}

