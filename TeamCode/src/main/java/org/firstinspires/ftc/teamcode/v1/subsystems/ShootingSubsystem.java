package org.firstinspires.ftc.teamcode.v1.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.v1.config.ShootingConfig;
import org.firstinspires.ftc.teamcode.v1.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.v1.services.MotorHelper;

public class ShootingSubsystem {

    private final RobotHardware hardware;
    private double[] pollenPidOutput = {0, 0, 0, 0};
    private double[] nectarPidOutput = {0, 0, 0, 0};
    public ShootingSubsystem(HardwareMap hardwareMap, RobotHardware hardware) {
        this.hardware = hardware;
    };
    public void initialize(){


    };

    public void startPollenShooter(double targetRPM){
        //hardware.getPollenLauncher().setPower(ShootingConfig.SHOOTING_POWER_POLLEN);

        if (hardware.getPollenLauncher().getPower() == 0) {
            pollenPidOutput = new double[]{0, 0, 0, 0};
        }

        pollenPidOutput = MotorHelper.pidRunWithEncoder(
                hardware.getPollenLauncher(),
                ShootingConfig.POLLEN_PGAIN,
                ShootingConfig.POLLEN_PGAIN2,
                ShootingConfig.POLLEN_IGAIN,
                targetRPM, //AI wanted it called targetRPM instead of power
                ShootingConfig.POLLEN_PDUTY_MIN,
                ShootingConfig.POLLEN_PDUTY_MAX,
                ShootingConfig.POLLEN_IDUTY_MIN,
                ShootingConfig.POLLEN_IDUTY_MAX,
                pollenPidOutput[1],
                ShootingConfig.POLLEN_POWER_MIN,
                ShootingConfig.POLLEN_POWER_MAX,
                ShootingConfig.POLLEN_GAIN,
                ShootingConfig.POLLEN_MIN_RPM,
                ShootingConfig.POLLEN_MAX_RPM,
                0,
                true
        );
        hardware.getPollenLauncher().setPower(-pollenPidOutput[0]);
    };

    public void startNectarShooter(double targetRPM){
        //hardware.getNectarLauncher().setPower(ShootingConfig.SHOOTING_POWER_NECTAR);

        if (hardware.getNectarLauncher().getPower() == 0) {
            nectarPidOutput = new double[]{0, 0, 0, 0};
        }

        nectarPidOutput = MotorHelper.pidRunWithEncoder(
                hardware.getNectarLauncher(),
                ShootingConfig.NECTAR_PGAIN,
                ShootingConfig.NECTAR_PGAIN2,
                ShootingConfig.NECTAR_IGAIN,
                targetRPM, //AI wanted it called targetRPM instead of power
                ShootingConfig.NECTAR_PDUTY_MIN,
                ShootingConfig.NECTAR_PDUTY_MAX,
                ShootingConfig.NECTAR_IDUTY_MIN,
                ShootingConfig.NECTAR_IDUTY_MAX,
                nectarPidOutput[1],
                ShootingConfig.NECTAR_POWER_MIN,
                ShootingConfig.NECTAR_POWER_MAX,
                ShootingConfig.NECTAR_GAIN,
                ShootingConfig.NECTAR_MIN_RPM,
                ShootingConfig.NECTAR_MAX_RPM,
                0,
                false
        );
        hardware.getNectarLauncher().setPower(-nectarPidOutput[0]);
    };

    public void pollenGatePosition(double position) {
        hardware.getPollengateServo().setPosition(position);
    }
    public void nectarGatePosition(double position) {
        hardware.getNectargateServo().setPosition(position);
    }

    public double getPollenLauncherRpm() {
        if (hardware.getPollenLauncher() == null) {
            return 0.0;
        }

        double ticksPerRev = hardware.getPollenLauncher().getMotorType().getTicksPerRev();
        double ticksPerSecond = Math.abs(hardware.getPollenLauncher().getVelocity());

        if (ticksPerRev <= 0) {
            return ticksPerSecond;
        }

        return (ticksPerSecond * 60.0) / ticksPerRev;
    }

    public double getNectarLauncherRpm() {
        if (hardware.getNectarLauncher() == null) {
            return 0.0;
        }

        double ticksPerRev = hardware.getNectarLauncher().getMotorType().getTicksPerRev();
        double ticksPerSecond = Math.abs(hardware.getNectarLauncher().getVelocity());

        if (ticksPerRev <= 0) {
            return ticksPerSecond;
        }

        return (ticksPerSecond * 60.0) / ticksPerRev;
    }


    public void stop(){
        hardware.getPollenLauncher().setPower(0);
        hardware.getNectarLauncher().setPower(0);
    };
}
