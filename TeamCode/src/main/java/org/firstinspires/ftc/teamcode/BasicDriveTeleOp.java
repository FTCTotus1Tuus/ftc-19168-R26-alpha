package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.List;

@TeleOp(name = "Drive - SDK Basic", group = "Bringup")
public class BasicDriveTeleOp extends LinearOpMode {

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private DriveHardwareResolver.Result resolvedNames;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing basic drivetrain...");
        telemetry.update();

        if (!initDrive()) {
            telemetry.addLine("Drive init failed. Check motor names in Robot Config.");
            if (resolvedNames != null && resolvedNames.notes != null && !resolvedNames.notes.isEmpty()) {
                telemetry.addData("Resolver", resolvedNames.notes);
            }
            telemetry.addData("Available motors", joinNames(", ", DriveHardwareResolver.listMotorNames(hardwareMap)));
            telemetry.addLine("Fix config names, then re-init this OpMode.");
            telemetry.update();

            while (!isStarted() && !isStopRequested()) {
                idle();
            }
            return;
        }

        telemetry.addLine("Basic drivetrain ready");
        telemetry.addData("LF/LR", "%s / %s", resolvedNames.leftFront, resolvedNames.leftRear);
        telemetry.addData("RF/RR", "%s / %s", resolvedNames.rightFront, resolvedNames.rightRear);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double lf = (y + x + rx) / denominator;
            double lr = (y - x + rx) / denominator;
            double rf = (y - x - rx) / denominator;
            double rr = (y + x - rx) / denominator;

            leftFront.setPower(Range.clip(lf, -1.0, 1.0));
            leftRear.setPower(Range.clip(lr, -1.0, 1.0));
            rightFront.setPower(Range.clip(rf, -1.0, 1.0));
            rightRear.setPower(Range.clip(rr, -1.0, 1.0));

            telemetry.addData("Mode", "SDK Basic Drive");
            telemetry.addData("LF/LR", "%.2f / %.2f", lf, lr);
            telemetry.addData("RF/RR", "%.2f / %.2f", rf, rr);
            telemetry.update();
            idle();
        }
    }

    private boolean initDrive() {
        try {
            resolvedNames = DriveHardwareResolver.resolve(hardwareMap);
            if (!resolvedNames.isComplete()) {
                return false;
            }

            leftFront = hardwareMap.get(DcMotorEx.class, resolvedNames.leftFront);
            leftRear = hardwareMap.get(DcMotorEx.class, resolvedNames.leftRear);
            rightFront = hardwareMap.get(DcMotorEx.class, resolvedNames.rightFront);
            rightRear = hardwareMap.get(DcMotorEx.class, resolvedNames.rightRear);

            leftFront.setDirection(DcMotor.Direction.FORWARD);
            leftRear.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            rightRear.setDirection(DcMotor.Direction.REVERSE);

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            return true;
        } catch (Throwable t) {
            RobotLog.ee("BasicDriveTeleOp", t, "Drivetrain init failed");
            telemetry.addData("Init Error", "%s: %s", t.getClass().getSimpleName(), t.getMessage());
            return false;
        }
    }

    private String joinNames(String separator, List<String> names) {
        if (names == null || names.isEmpty()) {
            return "<none>";
        }
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < names.size(); i++) {
            if (i > 0) {
                builder.append(separator);
            }
            builder.append(names.get(i));
        }
        return builder.toString();
    }
}



