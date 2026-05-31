package org.firstinspires.ftc.teamcode.v1.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * AutoConfig — autonomous start/target pose and timing margin placeholders.
 * Students should tune these on-field and keep names descriptive per route.
 */
@Config
public final class AutoConfig {
    private AutoConfig() {
    }

    // Example v1 start pose (inches, radians).
    public static double START_X_IN = 0.0;
    public static double START_Y_IN = 0.0;
    public static double START_HEADING_RAD = 0.0;

    // Example park pose (inches, radians).
    public static double PARK_X_IN = 24.0;
    public static double PARK_Y_IN = 0.0;
    public static double PARK_HEADING_RAD = 0.0;


    // Timing safety margins for autonomous sequencing.
    public static double ACTION_SETTLE_S = 0.15;
    public static double ENDGAME_BUFFER_S = 0.75;

    // TODO: add per-route constants here (e.g., V1_LEFT_PRELOAD_X, V1_RIGHT_SPIKE_Y).
}

