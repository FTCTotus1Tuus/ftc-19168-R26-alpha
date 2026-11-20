
package org.firstinspires.ftc.teamcode.v1.opmodes.auto;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.v1.config.ShootingConfig;
import org.firstinspires.ftc.teamcode.v1.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.v1.services.PedroPathingConstants;
import org.firstinspires.ftc.teamcode.v1.subsystems.ShootingSubsystem;

@Autonomous(name = "newAuto3", group = "Autonomous")
public class newAuto3 extends LinearOpMode {

    private Follower follower;

    private ShootingSubsystem shooting;

    private static final long SHOOTING_DURATION_MS = 7000;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start =
            poseFactory.of(58.5, 8.75, 90);

    private final Pose moveToIntake =
            poseFactory.of(58.5, 12, 90);

    private final Pose point2 =
            poseFactory.of(14, 95, 90);

    private final Pose point2Control1 =
            poseFactory.of(20.66, 12.5514, 0);

    // Original autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, moveToIntake()),
                follow(follower, path2())
        );
    }

    @Override
    public void runOpMode() {

        Scheduler.reset();

        // =====================================
        // INITIALIZE HARDWARE
        // =====================================

        RobotHardware hardware = new RobotHardware();
        hardware.initialize(hardwareMap);

        shooting = new ShootingSubsystem(hardwareMap, hardware);
        shooting.initialize();

        // Initialize original Pedro follower
        follower = PedroPathingConstants.createFollower(hardwareMap);
        follower.setPose(start);
        follower.update();

        // Close pollen gate before autonomous
        shooting.pollenGatePosition(
                ShootingConfig.POLLEN_GATE_CLOSE
        );

        telemetry.addLine("AutoPath READY");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            shooting.stop();
            return;
        }

        // =====================================
        // START POLLEN SHOOTER
        // =====================================

        shooting.startPollenShooter(
                ShootingConfig.TARGET_POLLEN_RPM
        );

        // =====================================
        // FIRST PATH: MOVE FORWARD
        // =====================================

        schedule(follow(follower, moveToIntake()));

        while (opModeIsActive() && follower.isBusy()) {

            follower.update();
            Scheduler.execute();

            // Maintain pollen shooter RPM
            shooting.startPollenShooter(
                    ShootingConfig.TARGET_POLLEN_RPM
            );

            telemetry.addData("State", "Moving Forward");
            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData("Heading", follower.pose().heading());

            telemetry.addData(
                    "Pollen RPM",
                    shooting.getPollenLauncherRpm()
            );

            telemetry.update();
        }

        if (!opModeIsActive()) {
            shooting.stop();
            return;
        }

        // =====================================
        // OPEN POLLEN GATE
        // =====================================

        shooting.pollenGatePosition(
                ShootingConfig.POLLEN_GATE_OPEN
        );

        long shootingStartTime = System.currentTimeMillis();

        // =====================================
        // SEVEN SECOND SHOOTING PAUSE
        // =====================================

        while (opModeIsActive()
                && System.currentTimeMillis() - shootingStartTime
                < SHOOTING_DURATION_MS) {

            follower.update();

            // Maintain shooter RPM during pause
            shooting.startPollenShooter(
                    ShootingConfig.TARGET_POLLEN_RPM
            );

            double elapsed =
                    (System.currentTimeMillis() - shootingStartTime)
                            / 1000.0;

            telemetry.addData("State", "Shooting - Gate Open");

            telemetry.addData(
                    "Pollen Timer",
                    "%.1f / 7.0 seconds",
                    elapsed
            );

            telemetry.addData(
                    "Pollen RPM",
                    shooting.getPollenLauncherRpm()
            );

            telemetry.update();

            idle();
        }

        if (!opModeIsActive()) {
            shooting.pollenGatePosition(
                    ShootingConfig.POLLEN_GATE_CLOSE
            );
            shooting.stop();
            return;
        }

        // =====================================
        // CLOSE POLLEN GATE
        // =====================================

        shooting.pollenGatePosition(
                ShootingConfig.POLLEN_GATE_CLOSE
        );

        // =====================================
        // SECOND PATH: BEZIER TO PARK
        // =====================================

        schedule(follow(follower, path2()));

        while (opModeIsActive() && follower.isBusy()) {

            follower.update();
            Scheduler.execute();

            // Maintain pollen shooter RPM
            shooting.startPollenShooter(
                    ShootingConfig.TARGET_POLLEN_RPM
            );

            telemetry.addData("State", "Bezier Parking");
            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData("Heading", follower.pose().heading());

            telemetry.addData(
                    "Pollen RPM",
                    shooting.getPollenLauncherRpm()
            );

            telemetry.update();
        }

        // =====================================
        // FINISHED
        // =====================================

        shooting.stop();

        telemetry.addLine("AUTONOMOUS FINISHED");
        telemetry.update();
    }

    // =====================================
    // ORIGINAL FIRST PATH
    // =====================================

    public Path moveToIntake() {
        return Paths.line(start, moveToIntake)
                .linear(start, moveToIntake);
    }

    // =====================================
    // ORIGINAL BEZIER PARKING PATH
    // =====================================

    public Path path2() {
        return Paths.curve(
                moveToIntake,
                point2Control1,
                point2
        ).constant(point2);
    }
}
