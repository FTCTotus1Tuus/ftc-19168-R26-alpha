package org.firstinspires.ftc.teamcode.v1.services;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.v1.config.DriveConfig;
import org.firstinspires.ftc.teamcode.v1.config.OdometryConfig;
import org.firstinspires.ftc.teamcode.v1.hardware.RobotHardwareNames;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;

public final class PedroPathingConstants {

    private PedroPathingConstants() {
    }

    public static Follower createFollower(HardwareMap hardwareMap) {

        // Apply the optional front-motor RPM limit.
        applyFrontGearCompensation(hardwareMap);

        // Foresight configuration.
        // These NONE values avoid incorrectly passing old Pedro 2.x
        // constraint values into Pedro 3's different constraint API.
        ForesightConfig foresightConfig = new ForesightConfig(c -> {
            c.maxVelocityConstraint.set(ForesightConfig.Constraint.NONE);
            c.maxAccelerationConstraint.set(ForesightConfig.Constraint.NONE);
            c.maxDecelerationConstraint.set(ForesightConfig.Constraint.NONE);
        });

        // Mecanum drivetrain configuration.
        MecanumConfig driveConfig = new MecanumConfig(c -> {
            c.frontLeftName.set(RobotHardwareNames.LEFT_FRONT_MOTOR);
            c.backLeftName.set(RobotHardwareNames.LEFT_REAR_MOTOR);
            c.frontRightName.set(RobotHardwareNames.RIGHT_FRONT_MOTOR);
            c.backRightName.set(RobotHardwareNames.RIGHT_REAR_MOTOR);

            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

            c.manualBrakeMode.set(true);
        });

        // goBILDA Pinpoint configuration.
        PinpointConfig localizerConfig = new PinpointConfig(c -> {
            c.name.set(RobotHardwareNames.PINPOINT);

            // The X port is for the forward-facing pod.
            // The Y port is for the strafe-facing pod.
            // Verify the offset mapping against your mounting and
            // the Pedro Pathing Pinpoint tuner results.
            c.xPodOffset.set(OdometryConfig.PINPOINT_FORWARD_POD_Y_IN);
            c.yPodOffset.set(OdometryConfig.PINPOINT_STRAFE_POD_X_IN);

            c.xPodDirection.set(
                    OdometryConfig.PINPOINT_FORWARD_ENCODER_REVERSED
                            ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                            : GoBildaPinpointDriver.EncoderDirection.FORWARD
            );

            c.yPodDirection.set(
                    OdometryConfig.PINPOINT_STRAFE_ENCODER_REVERSED
                            ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                            : GoBildaPinpointDriver.EncoderDirection.FORWARD
            );
        });

        // Pedro Pathing 3 follower construction.
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, driveConfig),
                new Foresight(foresightConfig)
        );
    }

    public static List<String> getAvailableMotorNames(HardwareMap hardwareMap) {
        LinkedHashSet<String> names = new LinkedHashSet<>();

        for (DcMotor motor : hardwareMap.getAll(DcMotor.class)) {
            names.addAll(hardwareMap.getNamesOf(motor));
        }

        return new ArrayList<>(names);
    }

    private static void applyFrontGearCompensation(HardwareMap hardwareMap) {
        if (!DriveConfig.GEAR_COMP_ENABLE_FRONT_RPM_LIMITING) {
            return;
        }

        double cappedFraction = Math.max(
                0.05,
                Math.min(
                        1.0,
                        DriveConfig.GEAR_COMP_FRONT_WHEEL_MAX_RPM_FRACTION
                )
        );

        applyMaxRpmFraction(
                hardwareMap,
                RobotHardwareNames.LEFT_FRONT_MOTOR,
                cappedFraction
        );

        applyMaxRpmFraction(
                hardwareMap,
                RobotHardwareNames.RIGHT_FRONT_MOTOR,
                cappedFraction
        );
    }

    private static void applyMaxRpmFraction(
            HardwareMap hardwareMap,
            String motorName,
            double maxRpmFraction
    ) {
        if (motorName == null) {
            return;
        }

        try {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorName);

            MotorConfigurationType motorType =
                    motor.getMotorType().clone();

            motorType.setAchieveableMaxRPMFraction(maxRpmFraction);
            motor.setMotorType(motorType);

        } catch (Exception ignored) {
            // Preserve the existing behavior of ignoring lookup failures.
        }
    }
}