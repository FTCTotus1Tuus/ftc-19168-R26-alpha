package org.firstinspires.ftc.teamcode.v1.testing;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.v1.services.PedroPathingConstants;

// Replaced by v1/opmodes/TeleOpMode.java — kept here as a reference skeleton.
// Remove @Disabled to temporarily restore this OpMode for diagnostics.
@Disabled
@TeleOp(name = "Teleop", group = "DriverControl")
public class PedroTeleOp extends LinearOpMode {

    private Follower follower;

    private boolean initFollower() {
        try {
            follower = PedroPathingConstants.createFollower(hardwareMap);
            return follower != null;
        } catch (Throwable t) {
            RobotLog.ee("PedroTeleOp", t, "Follower init failed");
            telemetry.addData("Init Error", "%s: %s", t.getClass().getSimpleName(), t.getMessage());
            telemetry.addData("Available motors", PedroPathingConstants.getAvailableMotorNames(hardwareMap).toString());
            return false;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Pedro teleop...");
        if (!initFollower()) {
            telemetry.addLine("Follower initialization failed");
            telemetry.update();

            while (!isStarted() && !isStopRequested()) {
                idle();
            }
            return;
        }

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        follower.startTeleopDrive(true);
        follower.update();

        while (opModeIsActive() && !isStopRequested()) {
            follower.setTeleOpDrive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x * 0.5,
                    true
            );
            follower.update();

            telemetry.update();
            idle();
        }
    }
}


