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

import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

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
            // Pedro 3 requires control/feedforward coefficients to be set.
            // Keep these aligned with the tuning constants currently used by
            // the Pedro diagnostic suite in v1/testing/pedroPathing.
            c.forwardTranslational.set(
                    Controller.piecewise(
                            Controller.proportional(0.1)
                    ).put(2.5, Controller.proportional(0.3))
            );

            c.strafeTranslational.set(
                    Controller.piecewise(
                            Controller.proportional(0.1)
                    ).put(2.5, Controller.proportional(0.3))
            );

            c.coast.set(Controller.proportionalFeedforward(0.010978350889324107));
            c.brake.set(Controller.proportionalFeedforward(0.008731598255925491));
            c.headingFeedback.set(Controller.proportional(5.258721785960744));

            c.headingBrakeCoefficients.set(
                    Vector2D.cartesian(
                            0.05642143125655298,
                            0.0063829525363003695
                    )
            );

            c.linearBrakeCoefficients.set(
                    Matrix.diag(
                            0.10605894992901523,
                            0.08719146175596092
                    )
            );

            c.quadraticBrakeCoefficients.set(
                    Matrix.diag(
                            0.0014663966976606565,
                            0.0013837064502458813
                    )
            );

            c.maxAchievableForwardVelocity.set(.01*72.72923108818539);
            c.maxAchievableStrafeVelocity.set(.01*52.34323936525474);
            c.naturalForwardDeceleration.set(.01*85.01144677379789);
            c.naturalStrafeDeceleration.set(.01*104.49787535782846);

            // Explicitly disable velocity/acceleration caps here since these
            // legacy values came from a different constraints API.
            c.maxVelocityConstraint.set(ForesightConfig.Constraint.NONE);
            c.maxAccelerationConstraint.set(ForesightConfig.Constraint.NONE);
            c.maxDecelerationConstraint.set(ForesightConfig.Constraint.NONE);
            c.maxPathSpeed.set(0.4);
        });

        // Mecanum drivetrain configuration.
        MecanumConfig driveConfig = new MecanumConfig(c -> {
            c.frontLeftName.set(RobotHardwareNames.LEFT_FRONT_MOTOR);
            c.backLeftName.set(RobotHardwareNames.LEFT_REAR_MOTOR);
            c.frontRightName.set(RobotHardwareNames.RIGHT_FRONT_MOTOR);
            c.backRightName.set(RobotHardwareNames.RIGHT_REAR_MOTOR);

            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);

            c.manualBrakeMode.set(true);

        });

        // goBILDA Pinpoint configuration.
        PinpointConfig localizerConfig = new PinpointConfig(c -> {
            c.name.set(resolvePinpointName(hardwareMap));

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

    private static String resolvePinpointName(HardwareMap hardwareMap) {
        String canonicalName = RobotHardwareNames.PINPOINT;

        if (hasNamedPinpoint(hardwareMap, canonicalName)) {
            return canonicalName;
        }

        // Common legacy name from earlier tuning/test setups.
        if (hasNamedPinpoint(hardwareMap, "odo")) {
            return "odo";
        }

        // Last-resort fallback: use the first available Pinpoint mapping name.
        for (GoBildaPinpointDriver pinpoint : hardwareMap.getAll(GoBildaPinpointDriver.class)) {
            Set<String> names = hardwareMap.getNamesOf(pinpoint);
            if (names != null && !names.isEmpty()) {
                return names.iterator().next();
            }
        }

        // Keep canonical as final fallback so error messaging remains predictable.
        return canonicalName;
    }

    private static boolean hasNamedPinpoint(HardwareMap hardwareMap, String name) {
        if (name == null || name.isEmpty()) {
            return false;
        }

        try {
            hardwareMap.get(GoBildaPinpointDriver.class, name);
            return true;
        } catch (Exception ignored) {
            return false;
        }
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