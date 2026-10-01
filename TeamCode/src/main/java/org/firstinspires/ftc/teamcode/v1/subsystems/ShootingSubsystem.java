package org.firstinspires.ftc.teamcode.v1.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.v1.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.v1.config.ShootingConfig;
import org.firstinspires.ftc.teamcode.v1.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.v1.services.PedroPathingConstants;
public class ShootingSubsystem {

    private final RobotHardware hardware;

    public ShootingSubsystem(HardwareMap hardwareMap, RobotHardware hardware) {
        this.hardware = hardware;
    };
    public void initialize(){


    };

    public void startPollenShooter(){
        hardware.getPollenLauncher().setPower(ShootingConfig.SHOOTING_POWER_POLLEN);
    };

    public void startNectarShooter(){
        hardware.getPollenLauncher().setPower(ShootingConfig.SHOOTING_POWER_NECTAR);
    };

    public void stop(){
        hardware.getPollenLauncher().setPower(0);
    };
}
