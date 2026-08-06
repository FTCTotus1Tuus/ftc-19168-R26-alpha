package org.firstinspires.ftc.teamcode.v1.opmodes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.v1.config.DriveConfig;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.v1.config.TeleOpMode_ballseekConfig;
import org.firstinspires.ftc.teamcode.v1.services.VisionService;

/**
 * TeleOpMode — driver-control OpMode.
 * Reads gamepad input and delegates to subsystems via RobotContainer.
 * Contains no hardware references — everything goes through robot.* fields.
 *
 * Loop order: clear bulk cache → update odometry → read input → queue drive → telemetry.
 */
@TeleOp(name = "V1 TeleOp_ballseek 2", group = "v1")
public class TeleOpMode_ballseek extends RobotOpMode {

    // Field-centric mode toggle state and edge detection for gamepad1.back.
    private boolean isFieldCentric = false;
    private boolean prevBackPressed = false;
    private boolean isSeekMode = false;

    private boolean prevAButtonPressed = false;

    private enum IntakeStates {OFF, IN, OUT};

    private IntakeStates intakeState = IntakeStates.OFF;
    private boolean prevLeftBumperPressed = false;

    private boolean prevBallWasCentered = false;

    private boolean prevBallWasVisible = false;

    private boolean prevBallWasClose = false;
    private boolean isReturnToLocationAMode = false;

    private boolean autoForwardMode = false;
    private boolean isBallDetected = false;
    private boolean isRawBallDetected = false;
    private boolean isBallDetectionCandidate = false;

    private ElapsedTime autoForwardTimer;
    private ElapsedTime ballVisibleTimer;
    private ElapsedTime ballDetectConfirmTimer;
    private ElapsedTime deliverDwellTimer;

    private double forward = 0;
    private double turn = 0;
    private enum SeekState {
        SEEK_BALL,
        RETURN_TO_LOCATION_A,
        DWELL_AT_LOCATION_A
    }

    private SeekState seekState = SeekState.SEEK_BALL;


    private boolean prevStartPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        if (robot.drive.isAvailable()) {
            telemetry.addLine("Ready");
        } else {
            telemetry.addLine("Drive unavailable — check motor config and re-init.");
        }
        telemetry.update();
        autoForwardTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        ballVisibleTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        ballDetectConfirmTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        deliverDwellTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        waitForStart();
        if (isStopRequested()) {
            stopRobot();
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            // 1. Clear bulk cache — must be the very first call in the loop.
            robot.hardware.clearBulkCache();

            // 2. Advance the Pedro Pathing follower: ticks the Pinpoint localizer so
            //    getPose() is fresh, and applies the drive powers queued last iteration.
            //    This must run every loop — before any drive-command logic — so odometry
            //    is never silently skipped by an early return in a drive branch (seek,
            //    auto-forward, or future auto-park).
            robot.drive.update();

            // 3. Toggle field-centric on rising edge of back button.
            boolean backPressed = gamepad1.back;
            if (backPressed && !prevBackPressed) {
                isFieldCentric = !isFieldCentric;
            }
            prevBackPressed = backPressed;

            // 3.2 Reset odometry to origin (0, 0, 0°) on rising edge of start button.
            boolean startPressed = gamepad1.start;
            if (startPressed && !prevStartPressed) {
                robot.localization.resetToOrigin();
            }
            prevStartPressed = startPressed;

            // 3.5 Toggle seek mode on rising edge of A button.
            boolean aPressed = gamepad1.a;
            if (aPressed && !prevAButtonPressed) {
                isSeekMode = !isSeekMode;
                if (isSeekMode) {
                    intakeState = IntakeStates.IN;
                    seekState = SeekState.SEEK_BALL;
                }else {
                    intakeState = IntakeStates.OFF;
                    seekState = SeekState.SEEK_BALL;
                    isReturnToLocationAMode = false;
                    autoForwardMode = false;
                    resetBallTrackingState();
                    // A pressed mid-return is the driver's abort: break any in-progress
                    // path and re-enter teleop drive, or the sticks stay dead.
                    robot.drive.stop();
                    robot.drive.resumeTeleOpDrive();
                }

            }
            prevAButtonPressed = aPressed;

            if (gamepad1.left_bumper) {
                if (intakeState == IntakeStates.OFF) {
                    intakeState = IntakeStates.IN;

                } else if (intakeState == IntakeStates.IN) {
                    intakeState = IntakeStates.OFF;

                }
            }

            NormalizedColorSensor rightColorSensor = robot.hardware.getRightColorSensor();
            NormalizedColorSensor leftColorSensor = robot.hardware.getLeftColorSensor();
            if (rightColorSensor instanceof DistanceSensor && leftColorSensor instanceof DistanceSensor) {
                double rightDistanceCm = ((DistanceSensor) rightColorSensor).getDistance(DistanceUnit.CM);
                double leftDistanceCm = ((DistanceSensor) leftColorSensor).getDistance(DistanceUnit.CM);
                telemetry.addData("Right Color Dist", "%.1f cm", rightDistanceCm);
                telemetry.addData("Left Color Dist", "%.1f cm", leftDistanceCm);

                isRawBallDetected = rightDistanceCm < TeleOpMode_ballseekConfig.DETECT_TRIGGER_CM
                        || leftDistanceCm < TeleOpMode_ballseekConfig.DETECT_TRIGGER_CM;

                if (isRawBallDetected) {
                    if (!isBallDetectionCandidate) {
                        isBallDetectionCandidate = true;
                        ballDetectConfirmTimer.reset();
                    }
                    isBallDetected = ballDetectConfirmTimer.time() >= TeleOpMode_ballseekConfig.DETECT_CONFIRM_SEC;
                } else {
                    isBallDetectionCandidate = false;
                    isBallDetected = false;
                }
            }


            // 4. Read input and drive.

            // 5. Ball scanning: always run regardless of drive mode.
            VisionService.BallTarget target = robot.vision.getBallTarget();


            // time based movement when ball ready (visible, centered, and close)
            if (isSeekMode && seekState == SeekState.SEEK_BALL) {
                if (target.isVisible) {
                    ballVisibleTimer.reset();
                    prevBallWasVisible = true;
                    // Steering: proportional to horizontal error
                    turn = Range.clip(
                            target.normalizedXError * TeleOpMode_ballseekConfig.BALL_SEEK_TURN_KP,
                            -TeleOpMode_ballseekConfig.BALL_SEEK_TURN_MAX,
                            TeleOpMode_ballseekConfig.BALL_SEEK_TURN_MAX
                    );

                    // Approach: slow down as ball appears larger
                    double radiusError = TeleOpMode_ballseekConfig.BALL_TARGET_RADIUS_PX - target.radiusPx;
                    forward = Range.clip(
                            radiusError * TeleOpMode_ballseekConfig.BALL_SEEK_FORWARD_KP,
                            0.0,
                            TeleOpMode_ballseekConfig.BALL_SEEK_FORWARD_MAX
                    );


                    // If centered and close enough, stop
                    if (target.radiusPx >= TeleOpMode_ballseekConfig.BALL_TARGET_RADIUS_PX) {
                        forward = TeleOpMode_ballseekConfig.BALL_SEEK_FORWARD_MAX;
                        prevBallWasClose = true;
                    }
                    if (Math.abs(target.normalizedXError) <= TeleOpMode_ballseekConfig.BALL_CENTER_TOLERANCE) {
                        turn = 0.0;
                        prevBallWasCentered = true;
                    }
                    if (prevBallWasClose && prevBallWasCentered) {
                        autoForwardMode = true;
                        autoForwardTimer.reset();
                    }


                } else if (ballVisibleTimer.time() > TeleOpMode_ballseekConfig.BALL_VISIBLE_SEC) {
                    prevBallWasVisible = false;
                    prevBallWasCentered = false;
                    prevBallWasClose = false;
                    // Target lost: slow scan in place
                    forward = 0.0;
                    if (Math.signum(turn) == 0) {
                        turn = TeleOpMode_ballseekConfig.BALL_SEARCH_TURN;
                    } else {
                        turn = Math.signum(turn) * TeleOpMode_ballseekConfig.BALL_SEARCH_TURN;
                    }
                }

                if (autoForwardMode) {
                    // Stop rotating and move forward for as long as timer commands
                    if (autoForwardTimer.time() >= TeleOpMode_ballseekConfig.BALL_TIMER_SEC) {
                        autoForwardMode = false;
                    } else {
                        forward = TeleOpMode_ballseekConfig.BALL_SEEK_FORWARD_MAX;
                        turn = 0.0;
                    }
                }
            }

            // Transition: ball collected → start the return path ONCE, at this edge.
            // followPath() must not be re-issued every loop — restarting the path each
            // pass resets the follower's progress and the robot never actually drives.
            if (isSeekMode && seekState == SeekState.SEEK_BALL && isBallDetected) {
                Pose pathStartPose = robot.drive.getPose();
                if (pathStartPose != null) {
                    PathChain parkPath = robot.drive.pathBuilder()
                            .addPath(new BezierLine(
                                    new Pose(pathStartPose.getX(), pathStartPose.getY()),
                                    new Pose(TeleOpMode_ballseekConfig.LOCATION_A_X, TeleOpMode_ballseekConfig.LOCATION_A_Y)
                            ))
                            .setLinearHeadingInterpolation(pathStartPose.getHeading(), Math.toRadians(TeleOpMode_ballseekConfig.LOCATION_A_HEADING_DEG))
                            .build();
                    // holdEnd=false so isFollowing() goes false on arrival (matches AutonomousBase usage);
                    // with holdEnd=true the follower can report busy forever and DWELL is never reached.
                    robot.drive.followPath(parkPath, false);
                    seekState = SeekState.RETURN_TO_LOCATION_A;
                    isReturnToLocationAMode = true;
                    autoForwardMode = false;
                    forward = 0.0;
                    turn = 0.0;
                }
                // If pose is unavailable, stay in SEEK_BALL — no path without localization.
            }

            if (!isSeekMode)  {
                applyTeleOpDrive(
                        gamepad1.left_stick_y,   // forward  (FTC SDK: negative when stick pushed up)
                        gamepad1.left_stick_x,   // strafe   (scaled by DriveConfig.TELEOP_ROTATION_SCALE)
                        gamepad1.right_stick_x,  // turn     (scaled by DriveConfig.TELEOP_ROTATION_SCALE)
                        gamepad1.right_trigger,  // precision: full press → TELEOP_PRECISION_SCALE speed
                        DriveConfig.TELEOP_DRIVE_DEADZONE,
                        DriveConfig.TELEOP_INPUT_EXPONENT,
                        DriveConfig.TELEOP_SPEED_SCALE,
                        DriveConfig.TELEOP_SPEED_SCALE_TURN,
                        DriveConfig.TELEOP_ROTATION_SCALE,
                        DriveConfig.TELEOP_PRECISION_SCALE,
                        DriveConfig.TELEOP_FIELD_CENTRIC_IS_RED_ALLIANCE,
                        DriveConfig.TELEOP_FIELD_CENTRIC_RED_OFFSET_RAD,
                        DriveConfig.TELEOP_FIELD_CENTRIC_BLUE_OFFSET_RAD,
                        isFieldCentric,
                        robot.drive.getPose(),
                        false // TODO: wire this up when AutoParking is built
                );
                if (intakeState == IntakeStates.OFF) {
                    robot.intake.stop();
                } else if (intakeState == IntakeStates.IN) {
                    robot.intake.start();
                }

            }
            else {
                if (seekState == SeekState.SEEK_BALL) {
                    robot.drive.setTeleOpDrive(-forward, 0.0, turn);
                } else if (seekState == SeekState.RETURN_TO_LOCATION_A) {
                    // Path was started once at the detection transition. The per-loop
                    // robot.drive.update() at the top of the loop drives it — issue NO
                    // drive commands here or they fight the follower.
                    if (!robot.drive.isFollowing()) {
                        // Arrived: hand the wheels back to teleop control, then dwell.
                        robot.drive.resumeTeleOpDrive();
                        seekState = SeekState.DWELL_AT_LOCATION_A;
                        deliverDwellTimer.reset();
                    }
                } else if (seekState == SeekState.DWELL_AT_LOCATION_A) {
                    robot.drive.setTeleOpDrive(0.0, 0.0, 0.0);
                    if (deliverDwellTimer.time() >= TeleOpMode_ballseekConfig.DELIVER_DWELL_SEC) {
                        seekState = SeekState.SEEK_BALL;
                        isReturnToLocationAMode = false;
                        autoForwardMode = false;
                        forward = 0.0;
                        turn = 0.0;
                        resetBallTrackingState();
                    }
                }
                robot.intake.start();
            }

            telemetry.addData("Ball Visible", target.isVisible);
            telemetry.addData("Ball X Err", "%.3f", target.normalizedXError);
            telemetry.addData("Ball Radius", "%.1f px", target.radiusPx);
            telemetry.addData("Circularity", "%.3f", target.circularity);
            telemetry.addData("Circle Center X-Y", "(%.1f, %.1f)", target.circleFitX, target.circleFitY);



            // 99. Telemetry.
            Pose _pose = robot.drive.getPose();
            telemetry.addData("Pose", _pose == null ? "N/A"
                    : String.format("(%.2f, %.2f) %.2f°", _pose.getX(), _pose.getY(), Math.toDegrees(_pose.getHeading())));
            telemetry.addData("Odometry Reset", "gamepad1.start  |  status: " + robot.localization.getStatus());
            telemetry.addData("Drive OK",  robot.drive.isAvailable());
            double allianceOffsetRad = DriveConfig.TELEOP_FIELD_CENTRIC_IS_RED_ALLIANCE
                    ? DriveConfig.TELEOP_FIELD_CENTRIC_RED_OFFSET_RAD
                    : DriveConfig.TELEOP_FIELD_CENTRIC_BLUE_OFFSET_RAD;
            telemetry.addData(
                    "Field-Centric",
                    "%s | Alliance=%s | Offset=%.0f deg",
                    isFieldCentric ? "ON (gamepad1.back to toggle)" : "OFF (robot-centric)",
                    DriveConfig.TELEOP_FIELD_CENTRIC_IS_RED_ALLIANCE ? "RED" : "BLUE",
                    Math.toDegrees(allianceOffsetRad)
            );
            telemetry.addData("Centered", prevBallWasCentered);
            telemetry.addData("Close", prevBallWasClose);
            telemetry.addData("AutoForward", autoForwardMode);
            telemetry.addData("Seek State", seekState);
            telemetry.addData("Return To locationA", isReturnToLocationAMode);
            telemetry.addData("Ball Detect Raw", isRawBallDetected);
            telemetry.addData("Ball Detect Confirmed", isBallDetected);
            telemetry.addData("Ball Detect Candidate", isBallDetectionCandidate);
            telemetry.addData("Ball Detect Timer", ballDetectConfirmTimer.time());
            telemetry.addData("Deliver Dwell Timer", deliverDwellTimer.time());
            telemetry.addData("AutoForward Timer", autoForwardTimer.time());
            telemetry.addData("Ball Visible Timer", ballVisibleTimer.time());
            telemetry.addData("Precision", "%.0f%%", (1.0 - gamepad1.right_trigger * (1.0 - DriveConfig.TELEOP_PRECISION_SCALE)) * 100);
            telemetry.addData("RPM LF", "%.1f", robot.drive.getLeftFrontRpm());
            telemetry.addData("RPM LR", "%.1f", robot.drive.getLeftRearRpm());
            telemetry.addData("RPM RF", "%.1f", robot.drive.getRightFrontRpm());
            telemetry.addData("RPM RR", "%.1f", robot.drive.getRightRearRpm());
            double avgFrontRpm = (robot.drive.getLeftFrontRpm() + robot.drive.getRightFrontRpm()) / 2.0;
            double avgRearRpm = (robot.drive.getLeftRearRpm() + robot.drive.getRightRearRpm()) / 2.0;
            telemetry.addData("Front/Rear RPM Ratio", "%.3f", avgRearRpm > 0.1 ? avgFrontRpm / avgRearRpm : 0.0);
            telemetry.addData("Seek Mode", isSeekMode ? "ON (A to toggle)" : "OFF");
            telemetry.update();

            idle();
        }

        stopRobot();
    }

    private static double normalizeRadians(double angleRad) {
        while (angleRad > Math.PI) {
            angleRad -= 2.0 * Math.PI;
        }
        while (angleRad < -Math.PI) {
            angleRad += 2.0 * Math.PI;
        }
        return angleRad;
    }

    private Pose getLocationAPose() {
        return new Pose(
                TeleOpMode_ballseekConfig.LOCATION_A_X,
                TeleOpMode_ballseekConfig.LOCATION_A_Y,
                Math.toRadians(TeleOpMode_ballseekConfig.LOCATION_A_HEADING_DEG)
        );
    }

    private void resetBallTrackingState() {
        isRawBallDetected = false;
        isBallDetected = false;
        isBallDetectionCandidate = false;
        prevBallWasVisible = false;
        prevBallWasCentered = false;
        prevBallWasClose = false;
        ballDetectConfirmTimer.reset();
    }

    public void applyTeleOpDrive(
            double leftStickY,
            double leftStickX,
            double rightStickX,
            double rightTrigger,
            double deadzone,
            double inputExponent,
            double speedScale,
            double speedScaleTurn,
            double rotationScale,
            double precisionScale,
            boolean isRedAlliance,
            double redAllianceOffsetRad,
            double blueAllianceOffsetRad,
            boolean isFieldCentric,
            Pose robotPose,
            boolean isAutoParking
    ) {
        if (isAutoParking) {
            // Auto-park owns drive commands while active.
            return;
        }

        // Circular deadzone for the translation stick; keep turn deadzoning per-axis.
        // We treat the left stick as one 2D vector so diagonal inputs behave the same
        // as straight inputs. This avoids the "cross-shaped" deadzone you get when X
        // and Y are filtered separately.
        // magnitude = the stick's distance from center, measured with the Pythagorean
        // theorem (hypotenuse of the X/Y triangle).
        double magnitude = Math.hypot(leftStickX, leftStickY);
        // Once the stick is outside the deadzone, remap the remaining range so the
        // driver still gets the full 0.0 to 1.0 control span.
        double translationScale = (magnitude <= deadzone || magnitude == 0)
                ? 0
                : (magnitude - deadzone) / (1.0 - deadzone);
        // Convert the scaled magnitude back into X/Y components by keeping the same
        // direction, then shrinking or growing the vector with translationScale.
        double rawY = (translationScale == 0) ? 0 : (leftStickY / magnitude) * translationScale;
        double rawX = (translationScale == 0) ? 0 : (leftStickX / magnitude) * translationScale;
        double rawR = (Math.abs(rightStickX) <= deadzone) ? 0 : rightStickX;

        // Exponential shaping gives finer low-speed control while preserving full-range output.
        double shapedY = Math.signum(rawY) * Math.pow(Math.abs(rawY), inputExponent);
        double shapedX = Math.signum(rawX) * Math.pow(Math.abs(rawX), inputExponent);
        double shapedR = Math.signum(rawR) * Math.pow(Math.abs(rawR), inputExponent);

        // Reduce forward/strafe speed when turning so rotation doesn't overpower translation.
        double driveScale = (rawR != 0) ? speedScaleTurn : speedScale;

        // Precision mode: right_trigger blends smoothly from full speed (trigger at rest)
        // down to precisionScale (trigger fully pressed). Useful near the scoring zone.
        // At trigger = 0.0 → multiplier = 1.0 (no change).
        // At trigger = 1.0 → multiplier = precisionScale (e.g. 30% speed).
        double precisionMultiplier = 1.0 - rightTrigger * (1.0 - precisionScale);

        // Field-centric driving: rotate the translation input by the robot's current heading
        // so that stick-up always moves forward relative to the field, not the robot.
        // Turn (shapedR) stays absolute — always rotates the robot itself.
        double commandX = shapedX;
        double commandY = shapedY;
        if (isFieldCentric && robotPose != null) {
            double allianceOffsetRad = isRedAlliance ? redAllianceOffsetRad : blueAllianceOffsetRad;
            double heading = robotPose.getHeading() + allianceOffsetRad;
            // Rotate input (commandX, commandY) into the robot frame using Pedro heading convention.
            // This keeps stick-up consistently "away from drivers" even when the robot is sideways.
            commandX = shapedX * Math.cos(heading) - shapedY * Math.sin(heading);
            commandY = shapedX * Math.sin(heading) + shapedY * Math.cos(heading);
        }

        robot.drive.setTeleOpDrive(
                commandY * driveScale * precisionMultiplier,
                commandX * driveScale * precisionMultiplier,
                shapedR * rotationScale * precisionMultiplier
        );
    }

}
