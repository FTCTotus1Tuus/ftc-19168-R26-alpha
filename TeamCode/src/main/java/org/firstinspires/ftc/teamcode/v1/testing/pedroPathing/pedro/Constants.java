package org.firstinspires.ftc.teamcode.v1.testing.pedroPathing.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.controllers.Controller;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static MecanumConfig drivetrainConfig =
            new MecanumConfig(c -> {
                // Change these names to match your Robot Configuration.
                c.frontLeftName.set("lf");
                c.backLeftName.set("lr");
                c.frontRightName.set("rf");
                c.backRightName.set("rr");

                c.frontLeftDirection.set(
                        DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(
                        DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(
                        DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(
                        DcMotorSimple.Direction.FORWARD);

                c.manualBrakeMode.set(true);
            });

    public static PinpointConfig localizerConfig =
            new PinpointConfig(c -> {
                // Change this name to match your Robot Configuration.
                c.name.set("pinpoint");

                // Starting offsets in millimeters.
                // Verify these against your actual pod mounting.
                c.xPodOffset.set(2.187);
                c.yPodOffset.set(-4.572);

                c.xPodDirection.set(
                        GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(
                        GoBildaPinpointDriver.EncoderDirection.FORWARD);
            });

    public static ForesightConfig foresightConfig =
            new ForesightConfig(c -> {
                // Leave motion constraints unrestricted during initial
                // configuration; tune Foresight before autonomous use.
                c.maxVelocityConstraint.set(
                        ForesightConfig.Constraint.NONE);
                c.maxAccelerationConstraint.set(
                        ForesightConfig.Constraint.NONE);
                c.maxDecelerationConstraint.set(
                        ForesightConfig.Constraint.NONE);
            });

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(
                        hardwareMap, localizerConfig),
                new Mecanum(
                        hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
