package org.firstinspires.ftc.teamcode.v1.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * PedroPathingConfig — dashboard-tunable path follower and drivetrain limits.
 */
@Config
public final class PedroPathingConfig {
    private PedroPathingConfig() {
    }

    // Prefixes provide pseudo-groups in FTC Dashboard's flat field list.

    // FOLLOWER_*: model constants.
    public static double FOLLOWER_WEIGHT_IN_KG = 5.26; // kg; 11.6 lbs

    // DRIVE_*: mecanum drive limits.
    public static double DRIVE_MAX_POWER = 1.0;
    public static double DRIVE_X_VELOCITY = 33; // at DRIVE_MAX_POWER = 0.5; see the Forward Velocity Tuner
    public static double DRIVE_Y_VELOCITY = 23; // at DRIVE_MAX_POWER = 0.5; see the Lateral Velocity Tuner

    // PATH_*: path constraint defaults.
    public static double PATH_TRANSLATIONAL_CONSTRAINT = 0.99;
    public static double PATH_HEADING_CONSTRAINT = 100;
    public static double PATH_TRANSLATIONAL_ACCEL_CONSTRAINT = 1;
    public static double PATH_HEADING_ACCEL_CONSTRAINT = 1;
}


