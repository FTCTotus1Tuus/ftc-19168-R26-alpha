package org.firstinspires.ftc.teamcode.v1.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShootingConfig {
    public static double SHOOTING_POWER_POLLEN = 0.3;
    public static double SHOOTING_POWER_NECTAR = 0.3;

    //Pollen PID constants
    public static double POLLEN_PGAIN = 0.002;
    public static double POLLEN_PGAIN2 = 0.0005;
    public static double POLLEN_IGAIN = 0.00003;
    public static double POLLEN_PDUTY_MIN = -0.5; // POLLEN_PDUTY_MIN = -0.5 may cause the PID to brake the motor if it overshoots — consider setting it to 0.0 for a flywheel since you never want reverse braking.
    public static double POLLEN_PDUTY_MAX = 1;
    public static double POLLEN_IDUTY_MIN = 0;
    public static double POLLEN_IDUTY_MAX = 1;
    public static double POLLEN_POWER_MIN = 0;
    public static double POLLEN_POWER_MAX = 1;
    public static double POLLEN_GAIN = 1;
    public static double POLLEN_MIN_RPM = 0;
    public static double POLLEN_MAX_RPM = 3000;
    public static double TARGET_POLLEN_RPM = 1000;

    //Nectar PID constants
    public static double NECTAR_PGAIN = 0.002;
    public static double NECTAR_PGAIN2 = 0.0005;
    public static double NECTAR_IGAIN = 0.00003;
    public static double NECTAR_PDUTY_MIN = -0.5; // POLLEN_PDUTY_MIN = -0.5 may cause the PID to brake the motor if it overshoots — consider setting it to 0.0 for a flywheel since you never want reverse braking.
    public static double NECTAR_PDUTY_MAX = 1;
    public static double NECTAR_IDUTY_MIN = 0;
    public static double NECTAR_IDUTY_MAX = 1;
    public static double NECTAR_POWER_MIN = 0;
    public static double NECTAR_POWER_MAX = 1;
    public static double NECTAR_GAIN = 1;
    public static double NECTAR_MIN_RPM = 0;
    public static double NECTAR_MAX_RPM = 3000;
    public static double TARGET_NECTAR_RPM = 1000;

    public static double[] pidOutput = {0, 0, 0, 0};

}
