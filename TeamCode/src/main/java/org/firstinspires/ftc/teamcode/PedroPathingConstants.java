package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;

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

    private static final String[] LEFT_FRONT_CANDIDATES = {
            "Left-front", "leftFront", "frontLeft", "left_front", "lf"
    };
    private static final String[] LEFT_REAR_CANDIDATES = {
            "Left-rear", "leftRear", "rearLeft", "left_rear", "lr"
    };
    private static final String[] RIGHT_FRONT_CANDIDATES = {
            "Right-front", "rightFront", "frontRight", "right_front", "rf"
    };
    private static final String[] RIGHT_REAR_CANDIDATES = {
            "Right-rear", "rightRear", "rearRight", "right_rear", "rr"
    };

    public static Follower createFollower(HardwareMap hardwareMap) {
        resolveConfiguredMotorNames(hardwareMap);

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

    public static List<String> getAvailableMotorNames(HardwareMap hardwareMap) {
        LinkedHashSet<String> names = new LinkedHashSet<>();
        for (DcMotor motor : hardwareMap.getAll(DcMotor.class)) {
            names.addAll(hardwareMap.getNamesOf(motor));
        }
        return new ArrayList<>(names);
    }

    private static void resolveConfiguredMotorNames(HardwareMap hardwareMap) {
        List<String> available = getAvailableMotorNames(hardwareMap);
        if (available.isEmpty()) {
            return;
        }

        Set<String> availableSet = new LinkedHashSet<>(available);
        Set<String> used = new LinkedHashSet<>();

        String lf = pickName(availableSet, used, LEFT_FRONT_CANDIDATES, "left", "front");
        String lr = pickName(availableSet, used, LEFT_REAR_CANDIDATES, "left", "rear");
        String rf = pickName(availableSet, used, RIGHT_FRONT_CANDIDATES, "right", "front");
        String rr = pickName(availableSet, used, RIGHT_REAR_CANDIDATES, "right", "rear");

        if (lf != null) {
            leftFrontMotorName = lf;
        }
        if (lr != null) {
            leftRearMotorName = lr;
        }
        if (rf != null) {
            rightFrontMotorName = rf;
        }
        if (rr != null) {
            rightRearMotorName = rr;
        }
    }

    private static String pickName(Set<String> available, Set<String> used, String[] candidates, String sideHint, String endHint) {
        for (String candidate : candidates) {
            String match = findCaseInsensitive(available, candidate);
            if (match != null && !used.contains(match)) {
                used.add(match);
                return match;
            }
        }

        for (String name : available) {
            if (used.contains(name)) {
                continue;
            }
            String lowered = name.toLowerCase(Locale.US);
            if (lowered.contains(sideHint) && lowered.contains(endHint)) {
                used.add(name);
                return name;
            }
        }

        return null;
    }

    private static String findCaseInsensitive(Set<String> available, String expected) {
        for (String name : available) {
            if (name.equalsIgnoreCase(expected)) {
                return name;
            }
        }
        return null;
    }

}
