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
// Ball detection + seek tuning
    public static double BALL_MIN_CONTOUR_AREA_PX = 20; //was 150
    public static double BALL_MAX_CONTOUR_AREA_PX = 20000;
    public static double BALL_MIN_CIRCULARITY = 0.55;

    public static int BALL_CAMERA_WIDTH_PX = 320;
    public static double BALL_TARGET_RADIUS_PX = 55;
    public static double BALL_CENTER_TOLERANCE = 0.08;

    public static double BALL_SEEK_TURN_KP = 0.60;
    public static double BALL_SEEK_FORWARD_KP = 0.020;
    public static double BALL_SEEK_TURN_MAX = 0.40;
    public static double BALL_SEEK_FORWARD_MAX = 0.35;
    public static double BALL_SEARCH_TURN = 0.20;

}
