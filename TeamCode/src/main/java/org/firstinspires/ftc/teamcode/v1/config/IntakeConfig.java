package org.firstinspires.ftc.teamcode.v1.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConfig {
    public static double INTAKE_POWER = -0.7;
    public static double INTAKE_POWER_SLOW = -0.2;
    public static double INTAKE_PGAIN = 0.002;
    public static double INTAKE_PGAIN2 = 0.0005;
    public static double INTAKE_IGAIN = 0.0003;
    public static double INTAKE_PDUTY_MIN = -0.5; // POLLEN_PDUTY_MIN = -0.5 may cause the PID to brake the motor if it overshoots — consider setting it to 0.0 for a flywheel since you never want reverse braking.
    public static double INTAKE_PDUTY_MAX = 1;
    public static double INTAKE_IDUTY_MIN = 0;
    public static double INTAKE_IDUTY_MAX = 1;
    public static double INTAKE_POWER_MIN = 0;
    public static double INTAKE_POWER_MAX = 1;
    public static double INTAKE_GAIN = 1;
    public static double INTAKE_MIN_RPM = 0;
    public static double INTAKE_MAX_RPM = 3000;
    public static double TARGET_INTAKE_RPM_NORMAL = 60;
    public static double TARGET_INTAKE_RPM_SLOW = 11;
    public static double[] intakePidOutput = {0, 0, 0, 0};
    public static double FLOWER_SCOOP_UP= 0.1;
    public static double FLOWER_SCOOP_DOWN = 0.75;
}
