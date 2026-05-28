package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PedroPathingConstants {
    public static String rightFrontMotorName = RobotHardwareNames.RIGHT_FRONT_MOTOR;
    public static String rightRearMotorName = RobotHardwareNames.RIGHT_REAR_MOTOR;
    public static String leftRearMotorName = RobotHardwareNames.LEFT_REAR_MOTOR;
    public static String leftFrontMotorName = RobotHardwareNames.LEFT_FRONT_MOTOR;

    public static double maxPower = 1.0;
    public static double xVelocity = 40;
    public static double yVelocity = 40;

    public static double translationalConstraint = 0.99;
    public static double headingConstraint = 100;
    public static double translationalAccelConstraint = 1;
    public static double headingAccelConstraint = 1;

    // Conservative defaults for a basic mecanum frame; tune later with Pedro tuners.
    public static double forwardTicksToInches = 1.0;
    public static double strafeTicksToInches = 1.0;
    public static double turnTicksToInches = 1.0;
    public static double robotWidthInches = 14.0;
    public static double robotLengthInches = 14.0;

    public static Follower createFollower(HardwareMap hardwareMap) {
        FollowerConstants followerConstants = new FollowerConstants();
        PathConstraints pathConstraints = new PathConstraints(
                translationalConstraint,
                headingConstraint,
                translationalAccelConstraint,
                headingAccelConstraint
        );

        MecanumConstants driveConstants = new MecanumConstants()
                .maxPower(maxPower)
                .rightFrontMotorName(rightFrontMotorName)
                .rightRearMotorName(rightRearMotorName)
                .leftRearMotorName(leftRearMotorName)
                .leftFrontMotorName(leftFrontMotorName)
                .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                .xVelocity(xVelocity)
                .yVelocity(yVelocity);

        DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
                .leftFrontMotorName(leftFrontMotorName)
                .leftRearMotorName(leftRearMotorName)
                .rightFrontMotorName(rightFrontMotorName)
                .rightRearMotorName(rightRearMotorName)
                .forwardTicksToInches(forwardTicksToInches)
                .strafeTicksToInches(strafeTicksToInches)
                .turnTicksToInches(turnTicksToInches)
                .robotWidth(robotWidthInches)
                .robotLength(robotLengthInches);

        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .driveEncoderLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

}
