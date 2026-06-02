package org.firstinspires.ftc.teamcode.v1.opmodes.auto;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.v1.config.DriveForwardOneConfig;

/**
 * DriveForwardOneAuto — drives forward exactly one FTC tile (24 in) and stops.
 *
 * Uses Pedro Pathing for closed-loop path following via the DriveSubsystem API.
 * Motors are never touched directly; all motion flows through robot.drive.
 *
 * ── Coordinate note ────────────────────────────────────────────────────────────────────────────
 * Starting pose is (0, 0, heading=0). The path drives in the +X direction, which is the
 * robot's "forward" when facing Pedro heading=0 in this project setup. Adjust START_POSE and END_POSE once the
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
 *
 * Follow-up tuning order once the robot is closer to final match weight:
 * 1) Finish the normal battery/mechanism loadout and verify all four wheels share load as expected.
 * 2) Re-check straight-line distance on a fresh battery before touching heading behavior.
 * 3) Tune Pedro drive/translation response for repeatable one-tile moves.
 * 4) Only then revisit small end-heading drift that may currently be dominated by mecanum slip.
 */
@Autonomous(name = "V1 Drive Forward 1 Tile", group = "Auto-v1")
public class DriveForwardOneAuto extends AutonomousBase {

    // ── Tuning constants ─────────────────────────────────────────────────────────────────────────

    // Route-specific config keeps this auto tunable without bloating shared AutoConfig.
    private static Pose startPose() {
        return new Pose(
                DriveForwardOneConfig.START_X_IN,
                DriveForwardOneConfig.START_Y_IN,
                DriveForwardOneConfig.START_HEADING_RAD
        );
    }

    private static Pose endPose() {
        double endXIn = DriveForwardOneConfig.START_X_IN
                + (DriveForwardOneConfig.FORWARD_DISTANCE_IN * DriveForwardOneConfig.FORWARD_AXIS_SIGN_X);
        double endYIn = DriveForwardOneConfig.START_Y_IN + DriveForwardOneConfig.STRAFE_OFFSET_IN;
        return new Pose(endXIn, endYIn, DriveForwardOneConfig.START_HEADING_RAD);
    }

    // ── Path ─────────────────────────────────────────────────────────────────────────────────────

    private PathChain driveForwardPath;
    private Pose plannedStartPose;
    private Pose plannedEndPose;

    // ── AutonomousBase contract ───────────────────────────────────────────────────────────────────

    /**
     * Builds the drive-forward path.
     * Called after initRobot() so the Follower and DriveSubsystem are ready.
     */
    @Override
    protected void buildPath() {
        plannedStartPose = startPose();
        plannedEndPose = endPose();
        driveForwardPath = buildDriveOnlyStraightPath(plannedStartPose, plannedEndPose);
    }

    // ── OpMode entry point ────────────────────────────────────────────────────────────────────────

    @Override
    public void runOpMode() {
        initRobot();
        buildPath();

        showStraightDriveReadyTelemetry(
                "V1 Drive Forward 1 Tile — Ready",
                plannedStartPose,
                plannedEndPose
        );

        waitForStart();
        if (isStopRequested()) {
            stopRobot();
            return;
        }

        if (driveForwardPath == null) {
            telemetry.addLine("Drive/path unavailable. Check motor config and follower initialization.");
            telemetry.update();
            saveFinalState();
            stopRobot();
            return;
        }

        followDriveOnlyStraightPath(
                driveForwardPath,
                plannedStartPose,
                plannedEndPose,
                DriveForwardOneConfig.MAX_AUTO_TIME_S,
                DriveForwardOneConfig.MAX_TRAVEL_FROM_START_IN,
                DriveForwardOneConfig.END_PROGRESS_TOLERANCE_IN,
                DriveForwardOneConfig.END_SETTLE_TIME_S
        );

        // ── Finish ────────────────────────────────────────────────────────────────────────────────
        saveFinalState();
        stopRobot();
    }
}

