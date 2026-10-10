package org.firstinspires.ftc.teamcode.v1.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.v1.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.v1.config.ShootingConfig;
import org.firstinspires.ftc.teamcode.v1.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.v1.services.MotorHelper;
import org.firstinspires.ftc.teamcode.v1.services.PedroPathingConstants;
public class ShootingSubsystem {

    private final RobotHardware hardware;

    public ShootingSubsystem(HardwareMap hardwareMap, RobotHardware hardware) {
        this.hardware = hardware;
    };
    public void initialize(){


    };

    public void startPollenShooter(double power){
        //hardware.getPollenLauncher().setPower(ShootingConfig.SHOOTING_POWER_POLLEN);

        if (hardware.getPollenLauncher().getPower() == 0) {
            ShootingConfig.pidOutput = new double[]{0, 0, 0, 0};
        }

        ShootingConfig.pidOutput = MotorHelper.pidRunWithEncoder(
                hardware.getPollenLauncher(),
                ShootingConfig.POLLEN_PGAIN,
                ShootingConfig.POLLEN_PGAIN2,
                ShootingConfig.POLLEN_IGAIN,
                power,
                ShootingConfig.POLLEN_PDUTY_MIN,
                ShootingConfig.POLLEN_PDUTY_MAX,
                ShootingConfig.POLLEN_IDUTY_MIN,
                ShootingConfig.POLLEN_IDUTY_MAX,
                ShootingConfig.pidOutput[1],
                ShootingConfig.POLLEN_POWER_MIN,
                ShootingConfig.POLLEN_POWER_MAX,
                ShootingConfig.POLLEN_GAIN,
                ShootingConfig.POLLEN_MIN_RPM,
                ShootingConfig.POLLEN_MAX_RPM,
                0,
                true
        );
        hardware.getPollenLauncher().setPower(-ShootingConfig.pidOutput[0]);
    };

    public void startNectarShooter(){
        //hardware.getNectarLauncher().setPower(ShootingConfig.SHOOTING_POWER_NECTAR);
    };

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

    public void stop(){
        hardware.getPollenLauncher().setPower(0);
        //hardware.getNectarLauncher().setPower(0);
    };
}
