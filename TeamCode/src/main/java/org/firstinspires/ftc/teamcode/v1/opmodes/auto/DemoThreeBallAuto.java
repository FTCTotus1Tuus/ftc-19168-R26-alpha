package org.firstinspires.ftc.teamcode.v1.opmodes.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.v1.config.DemoThreeBallConfig;
import org.firstinspires.ftc.teamcode.v1.config.DriveForwardOneConfig;

@Autonomous(name = "V1 Demo 3 Ball", group = "Auto-v1")

public class DemoThreeBallAuto extends AutonomousBase{
    private static Pose startPose() {
        return new Pose(
                DemoThreeBallConfig.START_X_IN,
                DemoThreeBallConfig.START_Y_IN,
                DemoThreeBallConfig.START_HEADING_RAD
        );
    }

    private static Pose endPose() {
        double endXIn = DemoThreeBallConfig.START_X_IN
                + (DemoThreeBallConfig.FORWARD_DISTANCE_IN * DemoThreeBallConfig.FORWARD_AXIS_SIGN_X);
        double endYIn = DemoThreeBallConfig.START_Y_IN + DemoThreeBallConfig.STRAFE_OFFSET_IN;
        return new Pose(endXIn, endYIn, DemoThreeBallConfig.START_HEADING_RAD);
    }

    private Pose plannedStartPose;
    private Pose plannedEndPose;

    @Override
    protected void buildPath() {
        plannedStartPose = startPose();
        plannedEndPose = endPose();
    }

    @Override
    public void runOpMode() {
        initRobot();
        buildPath();

        showStraightDriveReadyTelemetry(
                "V1 Drive Forward 1 Tile — Ready",
                plannedStartPose,
                plannedEndPose
        );

        waitForStart();
        if (isStopRequested()) {
            stopRobot();
            return;
        }

        // ── Finish ────────────────────────────────────────────────────────────────────────────────
        saveFinalState();
        stopRobot();
    }

}
