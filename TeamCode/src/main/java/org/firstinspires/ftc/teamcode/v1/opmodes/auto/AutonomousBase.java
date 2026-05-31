package org.firstinspires.ftc.teamcode.v1.opmodes.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.v1.opmodes.RobotOpMode;
import org.firstinspires.ftc.teamcode.v1.services.PreferencesService;

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

    // ── Route building ───────────────────────────────────────────────────────────────────────────

    /**
     * Override in each concrete autonomous subclass to define the path for that starting position.
     * Called after initRobot() and before waitForStart().
     */
    protected abstract void buildPath();

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

