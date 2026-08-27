package org.firstinspires.ftc.teamcode.v1.opmodes.auto;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
                .addPath(new BezierLine(plannedStartPose, plannedRedCorner))
                .setLinearHeadingInterpolation(plannedStartPose.getHeading(), plannedRedCorner.getHeading())
                .build();
    }

    @Override
    public void runOpMode(){
        initRobot();
        buildPath();

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

        robot.drive.setMaxPower(0.5);
        robot.drive.followPath(pathOne, false);

        while (opModeIsActive() && robot.drive.isFollowing()) {
            robot.hardware.clearBulkCache();
            robot.drive.update();
        }

        // ── Finish ────────────────────────────────────────────────────────────────────────────────
        saveFinalState();
        stopRobot();
    }
}
