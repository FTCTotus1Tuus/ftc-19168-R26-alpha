package org.firstinspires.ftc.teamcode.v1.opmodes.auto;

import android.content.SharedPreferences;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.v1.config.TeleOpMode_ballseekConfig;

@Autonomous(name = "NewAuto2", group = "Auto-v2")
public class NewAuto2 extends AutonomousBase {
    private static Pose startPose() {
        return new Pose(
                24,
                8.5,
                0
        );
    }

    private static Pose moveRedCorner() {
        return new Pose(
                96,
                96,
                0
        );
    }

    private static Pose moveBlueCorner() {
        return new Pose(
                35,
                96,
                0
        );
    }

    private static Pose moveEndPos() {
        return new Pose(
                96,
                25,
                -90
        );
    }

    private Pose plannedStartPose;
    private Pose plannedRedCorner;
    private Pose plannedBlueCorner;
    private Pose plannedEndPose;
    private PathChain pathOne;
    private PathChain pathTwo;
    private PathChain pathThree;
    private int pathState;
    private Timer pathTimer, opmodeTimer;
    private int STANDARD_PATH_TIMEOUT = 5;

    private void seedLocalizationToPlannedStart() {
        GoBildaPinpointDriver pinpoint = robot.hardware.getPinpoint();
        if (pinpoint != null) {
            pinpoint.setPosition(new Pose2D(
                    DistanceUnit.INCH,
                    plannedStartPose.getX(),
                    plannedStartPose.getY(),
                    AngleUnit.RADIANS,
                    plannedStartPose.getHeading()
            ));
            pinpoint.update();
        }
        robot.drive.setPose(plannedStartPose);
        robot.drive.setStartingPose(plannedStartPose);
    }

    @Override
    protected void buildPath() {
        plannedStartPose = startPose();
        plannedRedCorner = moveRedCorner();
        plannedBlueCorner = moveBlueCorner();
        plannedEndPose = moveEndPos();

        if (!robot.drive.isAvailable() || robot.drive.pathBuilder() == null) {
            pathOne = null;
            return;
        }

        robot.drive.setStartingPose(plannedStartPose);
        pathOne = robot.drive.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(plannedStartPose.getX(), plannedStartPose.getY()),
                        new Pose(plannedRedCorner.getX(), plannedRedCorner.getY())
                ))
                .setLinearHeadingInterpolation(plannedStartPose.getHeading(), plannedRedCorner.getHeading())
                .build();

        pathTwo = robot.drive.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(plannedRedCorner.getX(), plannedRedCorner.getY()),
                        new Pose(plannedBlueCorner.getX(), plannedBlueCorner.getY())
                ))
                .setLinearHeadingInterpolation(plannedRedCorner.getHeading(), plannedBlueCorner.getHeading())
                .build();

        pathThree = robot.drive.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(plannedBlueCorner.getX(), plannedBlueCorner.getY()),
                        new Pose(plannedEndPose.getX(), plannedEndPose.getY())
                ))
                .setLinearHeadingInterpolation(plannedBlueCorner.getHeading(), plannedEndPose.getHeading())
                .build();
    }

    @Override
    public void runOpMode() {
        initRobot();
        buildPath();
        seedLocalizationToPlannedStart();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        while (opModeInInit() && !isStopRequested()) {
            robot.hardware.clearBulkCache();
            seedLocalizationToPlannedStart();
            robot.drive.update();
            telemetry.addData("Planned Start", plannedStartPose);
            telemetry.addData("Live Pose", robot.drive.getPose());
            telemetry.update();
            idle();
        }

        waitForStart();
        if (isStopRequested()) {
            stopRobot();
            return;
        }

        if (pathOne == null) {
            telemetry.addLine("Drive/path unavailable. Check motor config and follower initialization.");
            telemetry.update();
            saveFinalState();
            stopRobot();
            return;
        }

        telemetry.update();

        seedLocalizationToPlannedStart();

        // ── Wait For Start ────────────────────────────────────────────────────────────────────────────────
        while (opModeIsActive() && !isStopRequested()) {

            // Pedro follower must be updated every loop
            robot.drive.update();

            // Drive the state machine
            pathState = autonomousPathUpdate();

            // Panels/driver telemetry

            telemetry.addData("Path State", pathState);
            telemetry.addData("X", robot.drive.getPose().getX());
            telemetry.addData("Y", robot.drive.getPose().getY());
            telemetry.addData("Heading", robot.drive.getPose().getHeading());

            telemetry.update();
        }
    }

    public int autonomousPathUpdate() {
        telemetry.addData("PathState", pathState);
        telemetry.addData("FollowerBusy", robot.drive.isFollowing());
        //telemetry.addData("PathTimer", pathTimer.getElapsedTimeSeconds());

        switch (pathState) {
            case 0:
                if (!robot.drive.isFollowing() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) {
                    robot.drive.followPath(pathOne, true);
                    setPathState(pathState + 1);
                }
                break;

            case 1:
                if (!robot.drive.isFollowing() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) {
                    robot.drive.followPath(pathTwo, true);
                    setPathState(pathState + 1);
                }
                break;

            case 2:
                if (!robot.drive.isFollowing() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) {
                    robot.drive.followPath(pathThree, true);
                    setPathState(-1);
                }
                break;

            default:
                // -1 or any undefined state: do nothing, stay idle
                telemetry.addLine("Idle state (pathState = " + pathState + ")");
                break;
        }

        return pathState;
    }

    /**
     * Sets the path state and resets its timer.
     */
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

}
