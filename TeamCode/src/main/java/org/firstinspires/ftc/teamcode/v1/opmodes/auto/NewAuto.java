package org.firstinspires.ftc.teamcode.v1.opmodes.auto;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.v1.config.MoveToCoordinateConfig;

@Autonomous(name = "V1 Move To (96,96)", group = "Auto-v1")
public class NewAuto extends AutonomousBase {

    private PathChain moveToTargetPath;
    private Pose plannedStartPose;
    private Pose plannedEndPose;

    @Override
    protected void buildPath() {
        moveToTargetPath = null;
        plannedStartPose = null;
        plannedEndPose = null;

        Pose livePose = robot.drive.getPose();
        plannedStartPose = (livePose != null)
                ? livePose
                : new Pose(
                        MoveToCoordinateConfig.AUTO_FALLBACK_START_X_IN,
                        MoveToCoordinateConfig.AUTO_FALLBACK_START_Y_IN,
                        MoveToCoordinateConfig.AUTO_FALLBACK_START_HEADING_RAD
                );

        plannedEndPose = new Pose(
                MoveToCoordinateConfig.AUTO_TARGET_X_IN,
                MoveToCoordinateConfig.AUTO_TARGET_Y_IN,
                plannedStartPose.getHeading()
        );

        if (!robot.drive.isAvailable() || robot.drive.pathBuilder() == null) {
            return;
        }

        robot.drive.setStartingPose(plannedStartPose);
        PathBuilder builder = robot.drive.pathBuilder()
                .addPath(new BezierLine(plannedStartPose, plannedEndPose));
        builder.setConstantHeadingInterpolation(plannedStartPose.getHeading());
        moveToTargetPath = builder.build();
    }

    @Override
    public void runOpMode() {
        initRobot();

        while (!isStarted() && !isStopRequested()) {
            robot.hardware.clearBulkCache();
            robot.drive.update();

            Pose previewPose = robot.drive.getPose();
            telemetry.addLine("V1 Move To (96,96) - Ready");
            telemetry.addData("Target XY (in)", "(%.1f, %.1f)",
                    MoveToCoordinateConfig.AUTO_TARGET_X_IN,
                    MoveToCoordinateConfig.AUTO_TARGET_Y_IN);
            telemetry.addData("Localization", robot.localization.getStatus());
            telemetry.addData("Live Pose", previewPose);
            telemetry.addLine("Path is rebuilt at START from live pose.");
            telemetry.update();
        }

        if (isStopRequested()) {
            stopRobot();
            return;
        }

        robot.hardware.clearBulkCache();
        robot.drive.update();
        buildPath();

        if (moveToTargetPath == null) {
            telemetry.addLine("Drive/path unavailable. Check motor config and follower initialization.");
            telemetry.update();
            saveFinalState();
            stopRobot();
            return;
        }

        followDriveOnlyStraightPath(
                moveToTargetPath,
                plannedStartPose,
                plannedEndPose,
                MoveToCoordinateConfig.AUTO_MAX_TIME_S,
                MoveToCoordinateConfig.AUTO_MAX_TRAVEL_FROM_START_IN,
                MoveToCoordinateConfig.AUTO_END_PROGRESS_TOLERANCE_IN,
                MoveToCoordinateConfig.AUTO_END_SETTLE_TIME_S
        );

        saveFinalState();
        stopRobot();
    }
}
