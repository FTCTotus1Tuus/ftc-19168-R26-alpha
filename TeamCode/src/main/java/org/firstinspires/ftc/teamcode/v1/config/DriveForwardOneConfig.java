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
    // Defaults to red audience anchor; switch to a different anchor as needed.
    public static double START_X_IN = AutoConfig.RED_AUDIENCE_START_X_IN;
    public static double START_Y_IN = AutoConfig.RED_AUDIENCE_START_Y_IN;
    public static double START_HEADING_RAD = AutoConfig.RED_AUDIENCE_START_HEADING_RAD;

    // Route shape controls. Flip FORWARD_AXIS_SIGN_X to -1.0 if "forward" is currently reversed.
    public static double FORWARD_DISTANCE_IN = 18.0; // Tune this to hit the target distance (24 in)
    public static double FORWARD_AXIS_SIGN_X = -1.0;
    public static double STRAFE_OFFSET_IN = 0.0;

    // Safety watchdogs for early-season bring-up.
    // If localization drifts or axis signs are wrong, these stop motion before wall contact.
    public static double MAX_AUTO_TIME_S = 3.0;
    public static double MAX_TRAVEL_FROM_START_IN = 32.0;

    // Consider the path complete once forward progress reaches the target distance.
    // Keep this small so the robot does not linger near the endpoint.
    public static double END_SETTLE_TIME_S = 0.0;
    public static double END_PROGRESS_TOLERANCE_IN = 1.0;

    // This route intentionally uses the shared drive-only straight-path helper instead of full
    // heading-hold path following; that proved more stable during early-season Pedro bring-up.

    // TODO: add route-specific toggles here (e.g., holdEndPose, reversePath, reducedSpeedMode).
}

