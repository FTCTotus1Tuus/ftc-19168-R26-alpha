package org.firstinspires.ftc.teamcode.v1.opmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.v1.config.DriveConfig;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.v1.config.VisionConfig;
import org.firstinspires.ftc.teamcode.v1.services.VisionService;
/**
 * TeleOpMode — driver-control OpMode.
 * Reads gamepad input and delegates to subsystems via RobotContainer.
 * Contains no hardware references — everything goes through robot.* fields.
 *
 * Loop order: clear bulk cache → read input → update subsystems → telemetry.
 */
@TeleOp(name = "V1 TeleOp_ballseek", group = "v1")
public class TeleOpMode_ballseek extends RobotOpMode {

    // Field-centric mode toggle state and edge detection for gamepad1.back.
    private boolean isFieldCentric = false;
    private boolean prevBackPressed = false;
    private boolean isSeekMode = false;
    private boolean prevAButtonPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        if (robot.drive.isAvailable()) {
            telemetry.addLine("Ready");
        } else {
            telemetry.addLine("Drive unavailable — check motor config and re-init.");
        }
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            stopRobot();
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {

            // 1. Clear bulk cache — must be the very first call in the loop.
            robot.hardware.clearBulkCache();

            // 1.5 Toggle field-centric on rising edge of back button.
            boolean backPressed = gamepad1.back;
            if (backPressed && !prevBackPressed) {
                isFieldCentric = !isFieldCentric;
            }
            prevBackPressed = backPressed;

            // 1.6 Toggle seek mode on rising edge of A button.
            boolean aPressed = gamepad1.a;
            if (aPressed && !prevAButtonPressed) {
                isSeekMode = !isSeekMode;
            }
            prevAButtonPressed = aPressed;

            // 2. Read input and drive.
            if (!isSeekMode) {
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
            } else {
                //3. Ball seeking
                VisionService.BallTarget target = robot.vision.getBallTarget();

                double forward;
                double turn;

                if (!target.isVisible) {
                    // Target lost: slow scan in place
                    forward = 0.0;
                    turn = VisionConfig.BALL_SEARCH_TURN;
                } else {
                    // Steering: proportional to horizontal error
                    turn = Range.clip(
                            target.normalizedXError * VisionConfig.BALL_SEEK_TURN_KP,
                            -VisionConfig.BALL_SEEK_TURN_MAX,
                            VisionConfig.BALL_SEEK_TURN_MAX
                    );

                    // Approach: slow down as ball appears larger
                    double radiusError = VisionConfig.BALL_TARGET_RADIUS_PX - target.radiusPx;
                    forward = Range.clip(
                            radiusError * VisionConfig.BALL_SEEK_FORWARD_KP,
                            0.0,
                            VisionConfig.BALL_SEEK_FORWARD_MAX
                    );

                    // If centered and close enough, stop
                    if (Math.abs(target.normalizedXError) <= VisionConfig.BALL_CENTER_TOLERANCE
                            && target.radiusPx >= VisionConfig.BALL_TARGET_RADIUS_PX) {
                        forward = 0.0;
                        turn = 0.0;
                    }

                }

                //robot.drive.setTeleOpDrive(forward, 0.0, turn);//disabled to read the telemetry values without robot moving
                robot.drive.setTeleOpDrive(0, 0.0, 0);//re-enable when ready to test ball seeking

                telemetry.addData("Ball Visible", target.isVisible);
                telemetry.addData("Ball X Err", "%.3f", target.normalizedXError);
                telemetry.addData("Ball Radius", "%.1f px", target.radiusPx);
                telemetry.addData("Circularity", "%.3f", target.circularity);
                telemetry.addData("Circle Center X-Y", "(%.1f, %.1f)", target.circleFitX, target.circleFitY);
            }


            // 99. Telemetry.
            Pose _pose = robot.drive.getPose();
            telemetry.addData("Pose", _pose == null ? "N/A"
                    : String.format("(%.2f, %.2f) %.2f°", _pose.getX(), _pose.getY(), Math.toDegrees(_pose.getHeading())));
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
