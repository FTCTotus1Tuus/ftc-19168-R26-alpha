package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

@TeleOp(name = "Drive - Pedro Smoke Test", group = "Bringup")
public class PedroDriveSmokeTest extends LinearOpMode {

    private Object follower;
    private DriveHardwareResolver.Result resolvedNames;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Pedro smoke test init...");
        telemetry.update();

        if (!initFollower()) {
            telemetry.addLine("Follower init failed.");
            if (resolvedNames != null && resolvedNames.notes != null && !resolvedNames.notes.isEmpty()) {
                telemetry.addData("Resolver", resolvedNames.notes);
            }
            telemetry.addData("Available motors", DriveHardwareResolver.listMotorNames(hardwareMap).toString());
            telemetry.addLine("Use 'Drive - SDK Basic' as fallback.");
            telemetry.addLine("Fix config names, then re-init this OpMode.");
            telemetry.update();

            while (!isStarted() && !isStopRequested()) {
                idle();
            }
            return;
        }

        telemetry.addLine("Follower ready");
        telemetry.addData("LF/LR", "%s / %s", resolvedNames.leftFront, resolvedNames.leftRear);
        telemetry.addData("RF/RR", "%s / %s", resolvedNames.rightFront, resolvedNames.rightRear);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        if (!callFollower("startTeleopDrive", new Class<?>[]{boolean.class}, true)) {
            return;
        }

        while (opModeIsActive() && !isStopRequested()) {
            boolean ok = callFollower(
                    "setTeleOpDrive",
                    new Class<?>[]{double.class, double.class, double.class, boolean.class},
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x * 0.5,
                    true
            );

            ok = ok && callFollower("update", new Class<?>[]{});
            if (!ok) {
                break;
            }

            telemetry.addData("Mode", "Pedro Smoke Test");
            telemetry.addData("Status", "Running");
            telemetry.update();
            idle();
        }
    }

    private boolean initFollower() {
        try {
            resolvedNames = DriveHardwareResolver.resolve(hardwareMap);
            if (!resolvedNames.isComplete()) {
                return false;
            }

            PedroPathingConstants.leftFrontMotorName = resolvedNames.leftFront;
            PedroPathingConstants.leftRearMotorName = resolvedNames.leftRear;
            PedroPathingConstants.rightFrontMotorName = resolvedNames.rightFront;
            PedroPathingConstants.rightRearMotorName = resolvedNames.rightRear;

            Class<?> constantsClass = Class.forName("org.firstinspires.ftc.teamcode.PedroPathingConstants");
            Method createFollower = constantsClass.getMethod("createFollower", com.qualcomm.robotcore.hardware.HardwareMap.class);
            follower = createFollower.invoke(null, hardwareMap);
            return follower != null;
        } catch (InvocationTargetException e) {
            Throwable cause = e.getCause() == null ? e : e.getCause();
            RobotLog.ee("PedroDriveSmokeTest", cause, "Follower init failed");
            telemetry.addData("Init Error", "%s: %s", cause.getClass().getSimpleName(), cause.getMessage());
            return false;
        } catch (Throwable t) {
            RobotLog.ee("PedroDriveSmokeTest", t, "Follower init failed");
            telemetry.addData("Init Error", "%s: %s", t.getClass().getSimpleName(), t.getMessage());
            return false;
        }
    }

    private boolean callFollower(String methodName, Class<?>[] argTypes, Object... args) {
        if (follower == null) {
            telemetry.addLine("Follower unavailable");
            telemetry.update();
            return false;
        }

        try {
            Method method = follower.getClass().getMethod(methodName, argTypes);
            method.invoke(follower, args);
            return true;
        } catch (InvocationTargetException e) {
            Throwable cause = e.getCause() == null ? e : e.getCause();
            RobotLog.ee("PedroDriveSmokeTest", cause, "Follower call failed: %s", methodName);
            telemetry.addData("Follower Error", "%s: %s", cause.getClass().getSimpleName(), cause.getMessage());
            telemetry.update();
            return false;
        } catch (Throwable t) {
            RobotLog.ee("PedroDriveSmokeTest", t, "Follower call failed: %s", methodName);
            telemetry.addData("Follower Error", "%s (%s)", methodName, t.getClass().getSimpleName());
            telemetry.update();
            return false;
        }
    }
}




