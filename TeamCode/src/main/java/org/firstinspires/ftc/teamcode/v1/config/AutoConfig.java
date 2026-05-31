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

    // Shared start-pose scaffolding (inches, radians).
    // Tune these four anchors first, then route-specific configs can reference them.
    public static double RED_AUDIENCE_START_X_IN = 0.0;
    public static double RED_AUDIENCE_START_Y_IN = 0.0;
    public static double RED_AUDIENCE_START_HEADING_RAD = 0.0;

    public static double RED_BACK_START_X_IN = 0.0;
    public static double RED_BACK_START_Y_IN = 0.0;
    public static double RED_BACK_START_HEADING_RAD = 0.0;

    public static double BLUE_AUDIENCE_START_X_IN = 0.0;
    public static double BLUE_AUDIENCE_START_Y_IN = 0.0;
    public static double BLUE_AUDIENCE_START_HEADING_RAD = 0.0;

    public static double BLUE_BACK_START_X_IN = 0.0;
    public static double BLUE_BACK_START_Y_IN = 0.0;
    public static double BLUE_BACK_START_HEADING_RAD = 0.0;

    // Example park pose (inches, radians).
    public static double PARK_X_IN = 24.0;
    public static double PARK_Y_IN = 0.0;
    public static double PARK_HEADING_RAD = 0.0;


    // Timing safety margins for autonomous sequencing.
    public static double ACTION_SETTLE_S = 0.15;
    public static double ENDGAME_BUFFER_S = 0.75;

    // TODO: add shared autonomous tolerances here (e.g., pose epsilon, heading epsilon).
}

