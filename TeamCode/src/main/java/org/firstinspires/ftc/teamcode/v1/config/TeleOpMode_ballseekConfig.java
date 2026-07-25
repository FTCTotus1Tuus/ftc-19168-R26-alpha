package org.firstinspires.ftc.teamcode.v1.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TeleOpMode_ballseekConfig {
    public static double BALL_TARGET_RADIUS_PX = 100;
    public static double BALL_CENTER_TOLERANCE = 0.08;

    public static double BALL_SEEK_TURN_KP = 0.60;
    public static double BALL_SEEK_FORWARD_KP = 0.1;
    public static double BALL_SEEK_TURN_MAX = 0.40;
    public static double BALL_SEEK_FORWARD_MAX = 0.2;
    public static double BALL_SEARCH_TURN = 0.2;
    public static double BALL_TIMER_SEC = 2;
    public static double BALL_VISIBLE_SEC = 0.5;

    public static double BALL_RETURN_TRIGGER_DISTANCE_CM = 10.0;
    public static double BALL_RETURN_TRANSLATION_KP = 0.08;
    public static double BALL_RETURN_TRANSLATION_MAX = 0.35;
    public static double BALL_RETURN_HEADING_KP = 0.9;
    public static double BALL_RETURN_HEADING_MAX = 0.35;
    public static double BALL_RETURN_ORIGIN_TOLERANCE_IN = 1.5;
    public static double BALL_RETURN_HEADING_TOLERANCE_DEG = 6.0;
    public static double BALL_RETURN_REARM_DISTANCE_CM = 14.0;
}
