package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PedroPathingConstants {
    public static String rightFrontMotorName = "Right-front";
    public static String rightRearMotorName = "Right-rear";
    public static String leftRearMotorName = "Left-rear";
    public static String leftFrontMotorName = "Left-front";

    public static double maxPower = 1.0;
    public static double xVelocity = 40;
    public static double yVelocity = 40;

    public static double translationalConstraint = 0.99;
    public static double headingConstraint = 100;
    public static double translationalAccelConstraint = 1;
    public static double headingAccelConstraint = 1;

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

        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

}
