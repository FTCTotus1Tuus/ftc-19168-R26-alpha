package org.firstinspires.ftc.teamcode.v1.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            // 1. Clear bulk cache — must be the very first call in the loop.
            robot.hardware.clearBulkCache();

            // 2. Read input and drive.
            robot.drive.setTeleOpDrive(
                    gamepad1.left_stick_y,   // forward  (FTC SDK: negative when stick pushed up)
                    gamepad1.left_stick_x,   // strafe
                    gamepad1.right_stick_x   // turn     (scaled by DriveConfig.TURN_SCALE)
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
}
