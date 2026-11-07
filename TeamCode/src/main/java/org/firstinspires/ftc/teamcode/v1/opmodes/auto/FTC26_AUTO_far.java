package org.firstinspires.ftc.teamcode.v1.opmodes.auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "FTC26_AUTO_far", group = "Auto-v2")
public class FTC26_AUTO_far extends AutonomousBase {
    private static final double SIMPLE_FORWARD_DISTANCE_IN = 8.0;
    private static final double SIMPLE_FORWARD_POWER = 0.30;

    private Pose plannedStartPose;
    private Pose moveStartPose;
    private boolean isMoveComplete = false;

    private static Pose startPose() {
        return PoseFactory.degrees().of(58.5, 8.75, 90);
    }

    @Override
    protected void buildPath() {
        plannedStartPose = startPose();
    }

    private void seedLocalizationToPlannedStart() {
        GoBildaPinpointDriver pinpoint = robot.hardware.getPinpoint();
        if (pinpoint != null) {
            pinpoint.setPosition(new Pose2D(
                    DistanceUnit.INCH,
                    plannedStartPose.x(),
                    plannedStartPose.y(),
                    AngleUnit.RADIANS,
                    plannedStartPose.heading()
            ));
            pinpoint.update();
        }
        robot.drive.setPose(plannedStartPose);
        robot.drive.setStartingPose(plannedStartPose);
    }

    @Override
    public void runOpMode() {
        initRobot();
        buildPath();
        seedLocalizationToPlannedStart();

        while (opModeInInit() && !isStopRequested()) {
            robot.hardware.clearBulkCache();
            robot.drive.update();
            telemetry.addData("Drive Available", robot.drive.isAvailable());
            telemetry.addData("Live Pose", robot.drive.getPose());
            telemetry.update();
            idle();
        }

        waitForStart();
        if (isStopRequested()) {
            stopRobot();
            return;
        }

        if (!robot.drive.isAvailable()) {
            telemetry.addLine("Drive unavailable.");
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) {
                idle();
            }
            saveFinalState();
            stopRobot();
            return;
        }

        moveStartPose = robot.drive.getPose();
        isMoveComplete = false;

        while (opModeIsActive() && !isStopRequested()) {
            robot.hardware.clearBulkCache();
            robot.drive.update();
            runForwardOnlySequence();
            telemetry.addData("Move Complete", isMoveComplete);
            telemetry.addData("Pose", robot.drive.getPose());
            telemetry.update();
        }

        saveFinalState();
        stopRobot();
    }

    private void runForwardOnlySequence() {
        if (isMoveComplete) {
            robot.drive.setTeleOpDrive(0.0, 0.0, 0.0);
            robot.drive.stop();
            return;
        }

        Pose currentPose = robot.drive.getPose();
        if (moveStartPose == null && currentPose != null) {
            moveStartPose = currentPose;
        }

        if (currentPose == null || moveStartPose == null) {
            robot.drive.setTeleOpDrive(SIMPLE_FORWARD_POWER, 0.0, 0.0);
            return;
        }

        double distanceMovedInches = Math.hypot(
                currentPose.x() - moveStartPose.x(),
                currentPose.y() - moveStartPose.y()
        );
        if (distanceMovedInches >= SIMPLE_FORWARD_DISTANCE_IN) {
            robot.drive.setTeleOpDrive(0.0, 0.0, 0.0);
            robot.drive.stop();
            isMoveComplete = true;
            return;
        }

        robot.drive.setTeleOpDrive(SIMPLE_FORWARD_POWER, 0.0, 0.0);
    }
}
