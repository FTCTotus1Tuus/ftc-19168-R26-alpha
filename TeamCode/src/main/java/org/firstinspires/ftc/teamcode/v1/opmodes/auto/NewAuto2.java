package org.firstinspires.ftc.teamcode.v1.opmodes.auto;

import static com.pedropathing.api.Paths.line;

import android.content.SharedPreferences;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.utils.Timer;
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
        return PoseFactory.degrees().of(24, 8.5, 0);
    }

    private static Pose moveRedCorner() {
        return PoseFactory.degrees().of(96, 96, 0);
    }

    private static Pose moveBlueCorner() {
        return PoseFactory.degrees().of(35, 96, 0);
    }

    private static Pose moveEndPos() {
        return PoseFactory.degrees().of(96, 25, -90);
    }

    private Pose plannedStartPose;
    private Pose plannedRedCorner;
    private Pose plannedBlueCorner;
    private Pose plannedEndPose;
    private Path pathOne;
    private Path pathTwo;
    private Path pathThree;

    private final PoseFactory poses = PoseFactory.degrees();
    private int pathState;
    private Timer pathTimer, opmodeTimer;
    private int STANDARD_PATH_TIMEOUT = 5;

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
    protected void buildPath() {
        plannedStartPose = startPose();
        plannedRedCorner = moveRedCorner();
        plannedBlueCorner = moveBlueCorner();
        plannedEndPose = moveEndPos();

        if (!robot.drive.isAvailable()) {
            pathOne = null;
            pathTwo = null;
            pathThree = null;
            return;
        }

        robot.drive.setStartingPose(plannedStartPose);

        pathOne = line(
                plannedStartPose,
                plannedRedCorner
        ).linear(
                plannedStartPose,
                plannedRedCorner
        );

        pathTwo = line(
                plannedRedCorner,
                plannedBlueCorner
        ).linear(
                plannedRedCorner,
                plannedBlueCorner
        );

        pathThree = line(
                plannedBlueCorner,
                plannedEndPose
        ).linear(
                plannedBlueCorner,
                plannedEndPose
        );
    }

    @Override
    public void runOpMode() {
        initRobot();
        buildPath();
        seedLocalizationToPlannedStart();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.reset();

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
            telemetry.addData("X", robot.drive.getPose().x());
            telemetry.addData("Y", robot.drive.getPose().y());
            telemetry.addData("Heading", robot.drive.getPose().heading());

            telemetry.update();
        }
    }

    public int autonomousPathUpdate() {
        telemetry.addData("PathState", pathState);
        telemetry.addData("FollowerBusy", robot.drive.isFollowing());
        //telemetry.addData("PathTimer", pathTimer.getElapsedTimeSeconds());

        switch (pathState) {
            case 0:
                if (!robot.drive.isFollowing() || pathTimer.seconds() > STANDARD_PATH_TIMEOUT) {
                    robot.drive.followPath(pathOne);
                    setPathState(pathState + 1);
                }
                break;

            case 1:
                if (!robot.drive.isFollowing() || pathTimer.seconds() > STANDARD_PATH_TIMEOUT) {
                    robot.drive.followPath(pathTwo);
                    setPathState(pathState + 1);
                }
                break;

            case 2:
                if (!robot.drive.isFollowing() || pathTimer.seconds() > STANDARD_PATH_TIMEOUT) {
                    robot.drive.followPath(pathThree);
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
        pathTimer.reset();
    }

}
