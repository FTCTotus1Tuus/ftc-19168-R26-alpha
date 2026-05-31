package org.firstinspires.ftc.teamcode.v1.opmodes.auto;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * DriveForwardOneAuto — drives forward exactly one FTC tile (24 in) and stops.
 *
 * Uses Pedro Pathing for closed-loop path following via the DriveSubsystem API.
 * Motors are never touched directly; all motion flows through robot.drive.
 *
 * ── Coordinate note ────────────────────────────────────────────────────────────────────────────
 * Starting pose is (0, 0, heading=0). The path drives in the +Y direction, which is the
 * robot's "forward" when facing Pedro heading=0. Adjust START_POSE and END_POSE once the
 * alliance starting position is confirmed on the real field.
 * ───────────────────────────────────────────────────────────────────────────────────────────────
 *
 * Remove {@code @Disabled} when the starting pose has been confirmed on the field.
 *
 * Tiny validation checklist (run each time this auto is changed):
 * 1) Verify START_POSE and END_POSE match your field side and tile orientation.
 * 2) Run once with wheels off-ground to confirm drive direction and heading behavior.
 * 3) Run on-field at low risk; confirm it moves exactly one tile and stops cleanly.
 * 4) If drift is visible, retune Pedro constants before competition use.
 */
//@Disabled
@Autonomous(name = "V1 Drive Forward 1 Tile", group = "Auto-v1")
public class DriveForwardOneAuto extends AutonomousBase {

    // ── Tuning constants ─────────────────────────────────────────────────────────────────────────

    /** One FTC floor tile = 24 inches. */
    private static final double TILE_IN = 24.0;

    /**
     * Starting field pose. Update x/y/heading once the actual field position is known.
     * Heading is in radians (0 = facing Pedro +X axis).
     */
    private static final Pose START_POSE = new Pose(0, 0, 0);

    /**
     * End pose: 1 tile forward in the +Y direction while keeping heading constant.
     * Change the Y offset or use X if your robot's forward maps differently.
     */
    private static final Pose END_POSE = new Pose(0, TILE_IN, 0);

    // ── Path ─────────────────────────────────────────────────────────────────────────────────────

    private PathChain driveForwardPath;
    private boolean pathReady;

    // ── AutonomousBase contract ───────────────────────────────────────────────────────────────────

    /**
     * Builds the drive-forward path.
     * Called after initRobot() so the Follower and DriveSubsystem are ready.
     */
    @Override
    protected void buildPath() {
        pathReady = false;
        if (!robot.drive.isAvailable() || robot.drive.pathBuilder() == null) {
            return;
        }

        robot.drive.setStartingPose(START_POSE);

        driveForwardPath = robot.drive.pathBuilder()
                .addPath(new BezierLine(START_POSE, END_POSE))
                .setConstantHeadingInterpolation(START_POSE.getHeading())
                .build();
        pathReady = true;
    }

    // ── OpMode entry point ────────────────────────────────────────────────────────────────────────

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        buildPath();

        telemetry.addLine("V1 Drive Forward 1 Tile — Ready");
        telemetry.addData("Start", START_POSE);
        telemetry.addData("End  ", END_POSE);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            stopRobot();
            return;
        }

        if (!pathReady) {
            telemetry.addLine("Drive/path unavailable. Check motor config and follower initialization.");
            telemetry.update();
            saveFinalState();
            stopRobot();
            return;
        }

        // ── Execute path ──────────────────────────────────────────────────────────────────────────
        robot.drive.followPath(driveForwardPath, /* holdEnd= */ true);

        while (opModeIsActive() && robot.drive.isFollowing()) {
            robot.hardware.clearBulkCache();
            robot.drive.update();

            telemetry.addLine("Following path…");
            telemetry.addData("Pose", robot.drive.getPose());
            telemetry.update();
        }

        // ── Finish ────────────────────────────────────────────────────────────────────────────────
        saveFinalState();
        stopRobot();
    }
}

