package org.firstinspires.ftc.teamcode.v1.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * AimConfig — tuning for "hold gamepad1.left_bumper to rotate toward the goal AprilTag".
 * Constants only; the aim logic lives in TeleOpMode_ballseek.
 *
 * Error is the tag's horizontal offset in the camera image: -1 = far left edge,
 * 0 = dead center, +1 = far right edge.
 *
 * Starting values copy the ball-seek turn gains (which were tuned on this robot),
 * so the first test should feel similar to ball seek. Untested on the robot.
 */
@Config
public final class AimConfig {
    private AimConfig() {
    }

    // Turn power per unit of error. Too high → robot overshoots and wobbles; too low → sluggish.
    public static double AIM_TURN_KP = 0.60;

    // Never turn faster than this while aiming.
    public static double AIM_TURN_MAX = 0.40;

    // Smallest turn power used when off target, so the robot doesn't stall just short
    // of center due to friction. Set to 0 to disable.
    public static double AIM_TURN_MIN = 0.05;

    // "Close enough" band around center. 0.04 ≈ 1-2 degrees with a typical webcam.
    public static double AIM_CENTER_TOLERANCE = 0.04;

    // Shift the aim point if the camera isn't lined up with the shooter.
    // Positive = aim to the right of the tag. Tune by shooting, not by eye.
    public static double AIM_X_OFFSET = 0.0;
}
