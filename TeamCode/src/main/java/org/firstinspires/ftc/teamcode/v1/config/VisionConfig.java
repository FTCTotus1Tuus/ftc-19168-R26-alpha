package org.firstinspires.ftc.teamcode.v1.config;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.v1.hardware.RobotHardwareNames;

/**
 * VisionConfig — camera + AprilTag tuning constants.
 * Keep this class constants-only; vision behavior belongs in VisionService.
 */
@Config
public final class VisionConfig {
    private VisionConfig() {
    }

    // Camera name mirrors the canonical hardware name used by RobotHardware.
    public static String CAMERA_NAME = RobotHardwareNames.WEBCAM_NAME;

    // AprilTag IDs used in autonomous decisions (placeholder defaults).
    public static int APRILTAG_ID_LEFT = 1;
    public static int APRILTAG_ID_CENTER = 2;
    public static int APRILTAG_ID_RIGHT = 3;

    // Camera exposure/gain placeholders for future manual tuning.
    public static boolean USE_MANUAL_EXPOSURE = false;
    public static int EXPOSURE_MS = 8;
    public static int GAIN = 120;

    // Vision timeout defaults for fail-safe autonomous logic.
    public static double INIT_TIMEOUT_S = 1.0;
    public static double DETECTION_TIMEOUT_S = 0.5;

    // TODO: add season-specific vision thresholds (e.g., confidence gates, ROI limits).
}

