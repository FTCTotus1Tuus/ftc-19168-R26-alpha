package org.firstinspires.ftc.teamcode.v1.opmodes.auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.v1.config.ShootingConfig;

@Autonomous(name = "FTC26_AUTO_audience", group = "Auto-v2")
public class FTC26_AUTO_audience extends AutonomousBase {
    private static Pose startPose() {
       return PoseFactory.degrees().of(58.5, 8.75, 90);
    }

    private static Pose moveToIntake() { //end pos
        return PoseFactory.degrees().of(58.5, 16, 87);
    }
    private static Pose point2Start() {
        return PoseFactory.degrees().of(58.5, 11, 87);
    }
    private static Pose point2(){
        return PoseFactory.degrees().of(10, 95, 90);
    }
    private static Pose point2Control1() {
        return PoseFactory.degrees().of(16, 45.5, 0);
    }


    private Pose plannedStartPose;
    private Pose plannedIntakePose;
    private Pose plannedPoint2StartPose;
    private Pose plannedPoint2Pose;
    private Pose plannedPoint2Control1;
    private Path pathOne;
    private Path pathTwo;
    private final PoseFactory poses = PoseFactory.degrees();
    private int pathState;
    private Timer pathTimer;
    private int STANDARD_PATH_TIMEOUT = 5;
    private static final double PATH_START_SETTLE_SEC = 0.15;
    private static final double SHOOT_SPINUP_WAIT_SEC = 5.0;
    private static final double SHOOT_GATE_OPEN_DWELL_SEC = 1.2;
    private static final double SIMPLE_FORWARD_DISTANCE_IN = 2.0;
    private static final double SIMPLE_FORWARD_POWER = 0.30;
    private static final double SIMPLE_STRAFE_POWER = 0.45;
    private static final double SIMPLE_SHOOTER_SPINUP_SEC = 3.0;
    private static final double SIMPLE_SHOOT_OPEN_SEC = 2.0;
    private static final double SIMPLE_BETWEEN_MOVES_WAIT_SEC = 1.0;
    private static final double SIMPLE_STRAFE_LEFT_ONE_DISTANCE_IN = 30.0;
    private static final double SIMPLE_FORWARD_TWO_DISTANCE_IN = 80.0;
    private static final double SIMPLE_STRAFE_LEFT_TWO_DISTANCE_IN = 8.0;
    private static final double AUTO_LAUNCHER_TARGET_SCALE = 1;

    private enum SimpleAutoState {
        MOVE_FORWARD_AND_SPINUP,
        WAIT_FOR_SPINUP_TO_OPEN_GATES,
        SHOOT_WITH_GATES_OPEN,
        STRAFE_LEFT_48,
        WAIT_AFTER_STRAFE_LEFT_48,
        MOVE_FORWARD_80,
        WAIT_AFTER_MOVE_FORWARD_80,
        STRAFE_LEFT_8,
        DONE
    }

    private SimpleAutoState simpleAutoState = SimpleAutoState.MOVE_FORWARD_AND_SPINUP;
    private Pose simpleSegmentStartPose;

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
        plannedIntakePose = moveToIntake();
        plannedPoint2StartPose = point2Start();
        plannedPoint2Pose = point2();
        plannedPoint2Control1 = point2Control1();

        if (!robot.drive.isAvailable()) {
            pathOne = null;
            pathTwo = null;
            return;
        }

        robot.drive.setStartingPose(plannedStartPose);

        pathOne = line(plannedStartPose, plannedIntakePose).linear(plannedStartPose, plannedIntakePose);

        pathTwo = curve(plannedPoint2StartPose, plannedPoint2Control1, plannedPoint2Pose)
                .linear(plannedPoint2StartPose, plannedPoint2Pose);
    }

    private Path buildSecondPathFrom(Pose startPose) {
        Pose pathStart = (startPose != null) ? startPose : plannedPoint2StartPose;
        return curve(pathStart, plannedPoint2Control1, plannedPoint2Pose)
                .linear(pathStart, plannedPoint2Pose);
    }

    @Override
    public void runOpMode() {
        initRobot();
        buildPath();
        seedLocalizationToPlannedStart();

        pathTimer = new Timer();
        pathTimer.reset();

        while (opModeInInit() && !isStopRequested()) {
            robot.hardware.clearBulkCache();
            //seedLocalizationToPlannedStart();
            robot.drive.update();
            telemetry.addData("Drive Available", robot.drive.isAvailable());
            telemetry.addData("Drive Init Error", robot.drive.getInitError());
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
            telemetry.addLine("Drive/path unavailable. Fix follower init before running this auto.");
            telemetry.addData("Drive Available", robot.drive.isAvailable());
            telemetry.addData("Drive Init Error", robot.drive.getInitError());
            telemetry.update();

            while (opModeIsActive() && !isStopRequested()) {
                idle();
            }

            saveFinalState();
            stopRobot();
            return;
        }

        telemetry.update();

        //seedLocalizationToPlannedStart();
        setPathState(0);
        simpleAutoState = SimpleAutoState.MOVE_FORWARD_AND_SPINUP;
        simpleSegmentStartPose = robot.drive.getPose();
        pathTimer.reset();
        robot.shooting.pollenGatePosition(ShootingConfig.POLLEN_GATE_CLOSE);
        robot.shooting.nectarGatePosition(ShootingConfig.NECTAR_GATE_CLOSE);

        // ── Wait For Start ────────────────────────────────────────────────────────────────────────────────
        while (opModeIsActive() && !isStopRequested()) {

            // In MANUAL bulk-cache mode this must happen before any read/update work.
            robot.hardware.clearBulkCache();

            // Pedro follower must be updated every loop
            robot.drive.update();

            // Drive the state machine
            pathState = autonomousPathUpdate();

            // Panels/driver telemetry

            telemetry.addData("Path State", pathState);
            telemetry.addData("X", robot.drive.getPose().x());
            telemetry.addData("Y", robot.drive.getPose().y());
            telemetry.addData("Heading", robot.drive.getPose().heading());
            telemetry.addData("Configured Strafe Power", "%.2f", SIMPLE_STRAFE_POWER);
            telemetry.addData("Active Strafe Cmd", "%.2f", getActiveStrafeCommand());

            telemetry.update();
        }
    }

    public int autonomousPathUpdate() {
        telemetry.addData("PathState", pathState);
        telemetry.addData("FollowerBusy", robot.drive.isFollowing());
        telemetry.addData("SimpleAutoState", simpleAutoState);

        switch (simpleAutoState) {
            case MOVE_FORWARD_AND_SPINUP:
                runLaunchersAtAutoTarget();
                if (hasSegmentDistanceReached(SIMPLE_FORWARD_POWER, 0.0, SIMPLE_FORWARD_DISTANCE_IN)) {
                    robot.drive.setTeleOpDrive(0.0, 0.0, 0.0);
                    simpleAutoState = SimpleAutoState.WAIT_FOR_SPINUP_TO_OPEN_GATES;
                }
                break;

            case WAIT_FOR_SPINUP_TO_OPEN_GATES:
                runLaunchersAtAutoTarget();
                robot.drive.setTeleOpDrive(0.0, 0.0, 0.0);
                if (pathTimer.seconds() >= SIMPLE_SHOOTER_SPINUP_SEC) {
                    robot.shooting.pollenGatePosition(ShootingConfig.POLLEN_GATE_OPEN);
                    robot.shooting.nectarGatePosition(ShootingConfig.NECTAR_GATE_OPEN);
                    pathTimer.reset();
                    simpleAutoState = SimpleAutoState.SHOOT_WITH_GATES_OPEN;
                }
                break;

            case SHOOT_WITH_GATES_OPEN:
                runLaunchersAtAutoTarget();
                robot.drive.setTeleOpDrive(0.0, 0.0, 0.0);
                if (pathTimer.seconds() >= SIMPLE_SHOOT_OPEN_SEC) {
                    robot.shooting.pollenGatePosition(ShootingConfig.POLLEN_GATE_CLOSE);
                    robot.shooting.nectarGatePosition(ShootingConfig.NECTAR_GATE_CLOSE);
                    robot.shooting.stop();
                    simpleSegmentStartPose = null;
                    simpleAutoState = SimpleAutoState.STRAFE_LEFT_48;
                }
                break;

            case STRAFE_LEFT_48:
                if (hasSegmentDistanceReached(0.0, SIMPLE_STRAFE_POWER, SIMPLE_STRAFE_LEFT_ONE_DISTANCE_IN)) {
                    robot.drive.setTeleOpDrive(0.0, 0.0, 0.0);
                    simpleSegmentStartPose = null;
                    pathTimer.reset();
                    simpleAutoState = SimpleAutoState.WAIT_AFTER_STRAFE_LEFT_48;
                }
                break;

            case WAIT_AFTER_STRAFE_LEFT_48:
                robot.drive.setTeleOpDrive(0.0, 0.0, 0.0);
                if (pathTimer.seconds() >= SIMPLE_BETWEEN_MOVES_WAIT_SEC) {
                    simpleAutoState = SimpleAutoState.MOVE_FORWARD_80;
                }
                break;

            case MOVE_FORWARD_80:
                if (hasSegmentDistanceReached(SIMPLE_FORWARD_POWER, 0.0, SIMPLE_FORWARD_TWO_DISTANCE_IN)) {
                    robot.drive.setTeleOpDrive(0.0, 0.0, 0.0);
                    simpleSegmentStartPose = null;
                    pathTimer.reset();
                    simpleAutoState = SimpleAutoState.WAIT_AFTER_MOVE_FORWARD_80;
                }
                break;

            case WAIT_AFTER_MOVE_FORWARD_80:
                robot.drive.setTeleOpDrive(0.0, 0.0, 0.0);
                if (pathTimer.seconds() >= SIMPLE_BETWEEN_MOVES_WAIT_SEC) {
                    simpleAutoState = SimpleAutoState.STRAFE_LEFT_8;
                }
                break;

            case STRAFE_LEFT_8:
                if (hasSegmentDistanceReached(0.0, SIMPLE_STRAFE_POWER, SIMPLE_STRAFE_LEFT_TWO_DISTANCE_IN)) {
                    stopAllFunctions();
                    simpleAutoState = SimpleAutoState.DONE;
                    setPathState(-1);
                }
                break;

            case DONE:
            default:
                stopAllFunctions();
                break;
        }

        return pathState;
    }

    private double getActiveStrafeCommand() {
        if (simpleAutoState == SimpleAutoState.STRAFE_LEFT_48
                || simpleAutoState == SimpleAutoState.STRAFE_LEFT_8) {
            return SIMPLE_STRAFE_POWER;
        }
        return 0.0;
    }

    private void runLaunchersAtAutoTarget() {
        robot.shooting.startPollenShooter(ShootingConfig.TARGET_POLLEN_RPM * AUTO_LAUNCHER_TARGET_SCALE);
        robot.shooting.startNectarShooter(ShootingConfig.TARGET_NECTAR_RPM * AUTO_LAUNCHER_TARGET_SCALE);
    }

    private boolean hasSegmentDistanceReached(double forwardCommand, double strafeCommand, double distanceInches) {
        Pose currentPose = robot.drive.getPose();
        if (simpleSegmentStartPose == null && currentPose != null) {
            simpleSegmentStartPose = currentPose;
        }

        if (currentPose == null || simpleSegmentStartPose == null) {
            robot.drive.setTeleOpDrive(forwardCommand, strafeCommand, 0.0);
            return false;
        }

        double distanceMovedInches = Math.hypot(
                currentPose.x() - simpleSegmentStartPose.x(),
                currentPose.y() - simpleSegmentStartPose.y()
        );
        if (distanceMovedInches >= distanceInches) {
            return true;
        }

        robot.drive.setTeleOpDrive(forwardCommand, strafeCommand, 0.0);
        return false;
    }

    /*
    Legacy path-state logic disabled per request (cases 0-4):
    switch (pathState) {
        case 0:
            ...
        case 1:
            ...
        case 2:
            ...
        case 3:
            ...
        case 4:
            ...
        default:
            ...
    }
    */

    private void stopAllFunctions() {
        robot.drive.setTeleOpDrive(0.0, 0.0, 0.0);
        robot.drive.stop();
        robot.intake.stop();
        robot.shooting.pollenGatePosition(ShootingConfig.POLLEN_GATE_CLOSE);
        robot.shooting.nectarGatePosition(ShootingConfig.NECTAR_GATE_CLOSE);
        robot.shooting.stop();
    }

    /**
     * Sets the path state and resets its timer.
     */
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }

}
