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
}

