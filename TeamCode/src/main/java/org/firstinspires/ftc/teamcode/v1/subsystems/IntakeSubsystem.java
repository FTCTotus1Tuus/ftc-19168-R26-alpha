package org.firstinspires.ftc.teamcode.v1.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.v1.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.v1.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.v1.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.v1.services.MotorHelper;
import org.firstinspires.ftc.teamcode.v1.services.PedroPathingConstants;

public class IntakeSubsystem {
    private final RobotHardware hardware;

    public IntakeSubsystem(HardwareMap hardwareMap, RobotHardware hardware) {
        this.hardware = hardware;
    };
    public void initialize(){


    };

    public void speed(double targetRPM) {
        DcMotorEx intakeMotor = hardware.getIntakeMotor();

        if (intakeMotor != null) {
            if (hardware.getIntakeMotor().getPower() >= 0) {
                IntakeConfig.intakePidOutput = new double[]{0, 0, 0, 0};
            }

            IntakeConfig.intakePidOutput = MotorHelper.pidRunWithEncoder(
                    hardware.getIntakeMotor(),
                    IntakeConfig.INTAKE_PGAIN,
                    IntakeConfig.INTAKE_PGAIN2,
                    IntakeConfig.INTAKE_IGAIN,
                    targetRPM, //AI wanted it called targetRPM instead of power
                    IntakeConfig.INTAKE_PDUTY_MIN,
                    IntakeConfig.INTAKE_PDUTY_MAX,
                    IntakeConfig.INTAKE_IDUTY_MIN,
                    IntakeConfig.INTAKE_IDUTY_MAX,
                    IntakeConfig.intakePidOutput[1],
                    IntakeConfig.INTAKE_POWER_MIN,
                    IntakeConfig.INTAKE_POWER_MAX,
                    IntakeConfig.INTAKE_GAIN,
                    IntakeConfig.INTAKE_MIN_RPM,
                    IntakeConfig.INTAKE_MAX_RPM,
                    0,
                    false
            );
            hardware.getIntakeMotor().setPower(-IntakeConfig.intakePidOutput[0]);
        }
    };

    public void reverse() {
        DcMotorEx intakeMotor = hardware.getIntakeMotor();

        if (intakeMotor != null) {
            IntakeConfig.intakePidOutput = new double[]{0, 0, 0, 0};
            intakeMotor.setPower(Math.abs(IntakeConfig.INTAKE_POWER));
        }
    };

    public void stop(){
        DcMotorEx intakeMotor = hardware.getIntakeMotor();

        if (intakeMotor != null) {
            intakeMotor.setPower(0);
        }
    };

    public void start(){
        speed(IntakeConfig.TARGET_INTAKE_RPM_NORMAL);
    };

    public void scoopServoPos(double targetPos) {
        hardware.getfloweerscoopServo().setPosition(targetPos);
    }

    public double getIntakeRpm() {
        DcMotorEx intakeMotor = hardware.getIntakeMotor();
        if (intakeMotor == null) {
            return 0.0;
        }

        double ticksPerSecond = intakeMotor.getVelocity();
        double ticksPerRev = intakeMotor.getMotorType().getTicksPerRev();
        if (ticksPerRev <= 0.0) {
            return 0.0;
        }

        return ticksPerSecond * 60.0 / ticksPerRev;
    }
}
