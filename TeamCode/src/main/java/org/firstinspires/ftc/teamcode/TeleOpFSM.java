package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop", group = "DriverControl")
public class TeleOpFSM extends DarienOpModeFSM {

    @Override
    public boolean initControls() {
        return super.initControls();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("SDK-only TeleOp loaded");
        telemetry.addLine("Pedro follower is temporarily disabled for stability testing");
        telemetry.update();

        boolean initOk = initControls();
        if (!initOk) {
            telemetry.addLine("Init failed");
            telemetry.update();
            sleep(1500);
            return;
        }

        waitForStart();
        if (isStopRequested()) return;

        while (this.opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Left Y", gamepad1.left_stick_y);
            telemetry.addData("Left X", gamepad1.left_stick_x);
            telemetry.addData("Right X", gamepad1.right_stick_x);

            telemetry.update();
            idle();
        } //while opModeIsActive
    } //runOpMode

} //TeleOpFSM class
