package org.firstinspires.ftc.teamcode.v1.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.v1.config.DriveConfig;

/**
 * TeleOpMode — driver-control OpMode.
 * Reads gamepad input and delegates to subsystems via RobotContainer.
 * Contains no hardware references — everything goes through robot.* fields.
 *
 * Loop order: clear bulk cache → read input → update subsystems → telemetry.
 */
@TeleOp(name = "V1 TeleOp", group = "v1")
public class TeleOpMode extends RobotOpMode {

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

            // 2. Read input and drive.
            applyTeleOpDrive(
                    gamepad1.left_stick_y,   // forward  (FTC SDK: negative when stick pushed up)
                    gamepad1.left_stick_x,   // strafe
                    gamepad1.right_stick_x,   // turn     (scaled by DriveConfig.ROTATION_SCALE)
                    DriveConfig.DRIVE_DEADZONE,
                    DriveConfig.INPUT_EXPONENT,
                    DriveConfig.SPEED_SCALE,
                    DriveConfig.SPEED_SCALE_TURN,
                    DriveConfig.ROTATION_SCALE,
                    false // TODO: wire this up when AutoParking is built
            );

            // 3. Telemetry.
            telemetry.addData("Pose",     robot.drive.getPose());
            telemetry.addData("Drive OK", robot.drive.isAvailable());
            telemetry.addData("RPM LF", "%.1f", robot.drive.getLeftFrontRpm());
            telemetry.addData("RPM LR", "%.1f", robot.drive.getLeftRearRpm());
            telemetry.addData("RPM RF", "%.1f", robot.drive.getRightFrontRpm());
            telemetry.addData("RPM RR", "%.1f", robot.drive.getRightRearRpm());
            double avgFrontRpm = (robot.drive.getLeftFrontRpm() + robot.drive.getRightFrontRpm()) / 2.0;
            double avgRearRpm = (robot.drive.getLeftRearRpm() + robot.drive.getRightRearRpm()) / 2.0;
            telemetry.addData("Front/Rear RPM Ratio", "%.3f", avgRearRpm > 0.1 ? avgFrontRpm / avgRearRpm : 0.0);
            telemetry.update();

            idle();
        }

        stopRobot();
    }

    public void applyTeleOpDrive(
            double leftStickY,
            double leftStickX,
            double rightStickX,
            double deadzone,
            double inputExponent,
            double speedScale,
            double speedScaleTurn,
            double rotationScale,
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

        double driveScale = (rawR != 0) ? speedScaleTurn : speedScale;
        robot.drive.setTeleOpDrive(
                shapedY * driveScale,
                shapedX * driveScale,
                shapedR * rotationScale
        );
    }

}
