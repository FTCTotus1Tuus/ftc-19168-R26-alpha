package org.firstinspires.ftc.teamcode.v1.opmodes.auto;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.v1.opmodes.RobotOpMode;
import org.firstinspires.ftc.teamcode.v1.services.PreferencesService;

import java.util.Locale;

/**
 * AutonomousBase — shared base class for all v1 autonomous OpModes.
 * Extends RobotOpMode (and through it, LinearOpMode).
 *
 * Callers should use: buildPath() to define the autonomous route.
 * Concrete subclasses should NOT duplicate path-builder helpers — add them here.
 *
 * Startup flow:
 *   runOpMode()  (subclass)
 *     → initRobot()          — inherited: hardware + services init
 *     → buildPath()          — subclass defines the route
 *     → waitForStart()
 *     → execute path
 *     → saveFinalState()     — write pose/alliance for TeleOp handoff
 *     → stopRobot()          — inherited: clean shutdown
 */
public abstract class AutonomousBase extends RobotOpMode {

    private static final String TAG = "AutonomousBase";
    protected String allianceColor = PreferencesService.ALLIANCE_UNKNOWN;

    protected static final class StraightDriveResult {
        public final boolean safetyStopTriggered;
        public final String safetyReason;
        public final Pose finalPose;

        private StraightDriveResult(boolean safetyStopTriggered, String safetyReason, Pose finalPose) {
            this.safetyStopTriggered = safetyStopTriggered;
            this.safetyReason = safetyReason;
            this.finalPose = finalPose;
        }
    }

    // ── Route building ───────────────────────────────────────────────────────────────────────────

    /**
     * Override in each concrete autonomous subclass to define the path for that starting position.
     * Called after initRobot() and before waitForStart().
     */
    protected abstract void buildPath();

    /**
     * Builds a straight Pedro path and seeds the localizer with the supplied start pose.
     * Uses constant heading plus drive-only PID mode at execution time for stable straight moves.
     */
    protected PathChain buildDriveOnlyStraightPath(Pose startPose, Pose endPose) {
        if (!robot.drive.isAvailable() || robot.drive.pathBuilder() == null) {
            return null;
        }

        robot.drive.setStartingPose(startPose);

        PathBuilder pathBuilder = robot.drive.pathBuilder()
                .addPath(new BezierLine(startPose, endPose));
        pathBuilder.setConstantHeadingInterpolation(startPose.getHeading());
        return pathBuilder.build();
    }

    /** Adds a standard ready screen for straight-drive autonomous routes. */
    protected void showStraightDriveReadyTelemetry(String title, Pose startPose, Pose endPose, String... extraLines) {
        double deltaXIn = endPose.getX() - startPose.getX();
        double deltaYIn = endPose.getY() - startPose.getY();
        double pathDistanceIn = Math.hypot(deltaXIn, deltaYIn);
        boolean forwardDominant = Math.abs(deltaXIn) >= Math.abs(deltaYIn);

        telemetry.addLine(title);
        telemetry.addData("Start", startPose);
        telemetry.addData("End  ", endPose);
        telemetry.addData("Delta (in)", "dX=%.1f, dY=%.1f", deltaXIn, deltaYIn);
        telemetry.addData("Distance (in)", "%.1f", pathDistanceIn);
        telemetry.addData("Intent", forwardDominant ? "Forward-dominant (+/-X)" : "Strafe-dominant (+/-Y)");
        telemetry.addData("Heading mode", "constant %.1f deg (drive-only PID mode)", Math.toDegrees(startPose.getHeading()));
        if (!forwardDominant) {
            telemetry.addLine("Sanity check: This path is mostly strafing. Verify END_X/END_Y before start.");
        }
        for (String extraLine : extraLines) {
            telemetry.addLine(extraLine);
        }
        telemetry.update();
    }

    /**
     * Executes a straight drive-only Pedro path with common timeout/travel watchdogs.
     * Returns the final pose and whether execution ended via a safety stop.
     */
    protected StraightDriveResult followDriveOnlyStraightPath(
            PathChain path,
            Pose startPose,
            Pose endPose,
            double maxAutoTimeS,
            double maxTravelFromStartIn,
            double endProgressToleranceIn,
            double endSettleTimeS
    ) {
        if (path == null) {
            return new StraightDriveResult(true, "Drive/path unavailable", robot.drive.getPose());
        }

        robot.drive.enableDriveOnlyPathMode();
        robot.drive.followPath(path, false);

        ElapsedTime pathTimer = new ElapsedTime();
        ElapsedTime settleTimer = new ElapsedTime();
        boolean insideEndTolerance = false;
        boolean safetyStopTriggered = false;
        String safetyReason = "";

        double deltaXIn = endPose.getX() - startPose.getX();
        double deltaYIn = endPose.getY() - startPose.getY();
        double pathLengthIn = Math.max(1e-6, Math.hypot(deltaXIn, deltaYIn));
        double pathUnitX = deltaXIn / pathLengthIn;
        double pathUnitY = deltaYIn / pathLengthIn;

        while (opModeIsActive() && robot.drive.isFollowing()) {
            robot.hardware.clearBulkCache();
            robot.drive.update();

            Pose livePose = robot.drive.getPose();
            if (pathTimer.seconds() > maxAutoTimeS) {
                safetyStopTriggered = true;
                safetyReason = String.format(Locale.US, "Timeout %.2fs", pathTimer.seconds());
                robot.drive.stop();
                break;
            }

            if (livePose != null) {
                double traveledFromStartIn = Math.hypot(
                        livePose.getX() - startPose.getX(),
                        livePose.getY() - startPose.getY()
                );

                double translationErrorIn = Math.hypot(
                        endPose.getX() - livePose.getX(),
                        endPose.getY() - livePose.getY()
                );
                double progressIn = (livePose.getX() - startPose.getX()) * pathUnitX
                        + (livePose.getY() - startPose.getY()) * pathUnitY;

                if (traveledFromStartIn > maxTravelFromStartIn) {
                    safetyStopTriggered = true;
                    safetyReason = String.format(Locale.US, "Travel limit %.1f in", traveledFromStartIn);
                    robot.drive.stop();
                    break;
                }

                if (progressIn >= pathLengthIn - endProgressToleranceIn) {
                    if (!insideEndTolerance) {
                        settleTimer.reset();
                        insideEndTolerance = true;
                    }

                    if (settleTimer.seconds() >= endSettleTimeS) {
                        robot.drive.stop();
                        break;
                    }
                } else {
                    insideEndTolerance = false;
                }

                telemetry.addData("Err XY (in)", "%.2f", translationErrorIn);
                telemetry.addData("Progress (in)", "%.2f / %.2f", progressIn, pathLengthIn);
            }

            telemetry.addLine("Following path…");
            telemetry.addData("Pose", livePose);
            telemetry.addData("Elapsed (s)", "%.2f / %.2f", pathTimer.seconds(), maxAutoTimeS);
            telemetry.update();
        }

        Pose finalPose = robot.drive.getPose();
        if (safetyStopTriggered) {
            telemetry.addLine("SAFETY STOP triggered");
            telemetry.addData("Reason", safetyReason);
            telemetry.addData("Final Pose", finalPose);
            telemetry.addLine("Check odometry direction/sign if this triggers repeatedly.");
            telemetry.update();

            while (opModeIsActive() && !isStopRequested()) {
                idle();
            }
        }

        return new StraightDriveResult(safetyStopTriggered, safetyReason, finalPose);
    }

    /**
     * Optional helper for subclasses to declare alliance before saveFinalState().
     * Keep values constrained to PreferencesService.ALLIANCE_* constants.
     */
    protected void setAllianceColor(String allianceColor) {
        this.allianceColor = allianceColor;
    }

    // ── Post-autonomous handoff ──────────────────────────────────────────────────────────────────

    /**
     * Saves the final robot pose and alliance color via PreferencesService so TeleOp can
     * restore the correct field position after autonomous.
     * Call at the end of runOpMode(), before stopRobot().
     */
    protected void saveFinalState() {
        try {
            robot.preferences.saveAllianceColor(allianceColor);

            Pose finalPose = robot.drive.getPose();
            if (finalPose != null) {
                robot.preferences.savePose(finalPose);
            }

            // TODO: add additional handoff fields here when auto routes diverge by strategy.
        } catch (Throwable t) {
            RobotLog.ww(TAG, t, "Could not save final autonomous state; TeleOp will use default pose.");
        }
    }
}

