package org.firstinspires.ftc.teamcode.v1.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.v1.config.DriveConfig;
import org.firstinspires.ftc.teamcode.v1.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.v1.services.PedroPathingConstants;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * DriveSubsystem — wraps the Pedro Pathing Follower.
 * Callers should use: setTeleOpDrive(), followPath(), getPose(), stop().
 * Callers should NOT access the Follower or drive motors directly.
 *
 * If the Follower fails to initialize (e.g., motors not found in config),
 * all methods become safe no-ops. Check isAvailable() or getInitError()
 * to surface the problem in telemetry.
 */
public class DriveSubsystem {

    private static final String TAG = "DriveSubsystem";

    private final RobotHardware hardware;
    private Follower follower;
    private String initError = null;

    public DriveSubsystem(HardwareMap hardwareMap, RobotHardware hardware) {
        this.hardware = hardware;
        try {
            follower = PedroPathingConstants.createFollower(hardwareMap);
        } catch (Throwable t) {
            RobotLog.ee(TAG, t, "Follower init failed");
            initError = t.getClass().getSimpleName() + ": " + t.getMessage();
            follower = null;
        }
    }

    // ── Lifecycle ────────────────────────────────────────────────────────────────────────────────

    /**
     * Starts teleop drive mode on the Follower.
     * Call once after waitForStart(); only needed when isAvailable() is true.
     */
    public void initialize() {
        if (follower != null) {
            follower.startTeleopDrive(true);
            follower.update();
        }
    }

    /**
     * Re-enters TeleOp drive mode after autonomous path following completes.
     * Call once when switching back from a followPath() sequence so that
     * setTeleOpDrive() inputs are accepted again.
     */
    public void resumeTeleOpDrive() {
        if (follower != null) {
            follower.startTeleopDrive(true);
        }
    }

    /** Stops the follower and halts all drive motors. */
    public void stop() {
        if (follower != null) {
            follower.breakFollowing();
        }
    }

    // ── TeleOp ───────────────────────────────────────────────────────────────────────────────────

    /**
     * Queues robot-centric mecanum drive targets for the next follower update.
     * Does NOT advance the control loop — call {@link #update()} once per loop to tick
     * the localizer and apply the queued motor powers.
     *
     * @param forward  left_stick_y  — negative when stick is pushed up (FTC SDK convention)
     * @param strafe   left_stick_x  — positive when stick is pushed right
     * @param turn     right_stick_x — scaled by DriveConfig.TELEOP_ROTATION_SCALE for driver precision
     */
    public void setTeleOpDrive(double forward, double strafe, double turn) {
        if (follower != null) {
            follower.setTeleOpDrive(forward, strafe, turn * DriveConfig.TELEOP_ROTATION_SCALE, true);
        }
    }

    // ── Autonomous ───────────────────────────────────────────────────────────────────────────────

    /**
     * Seeds the Pedro Pathing localizer with the robot's known starting pose.
     * Call before followPath() so the Follower begins tracking from the correct field position.
     * Do not call after the robot has moved.
     *
     * @param pose the field pose (x, y, headingRadians) at the start of autonomous.
     */
    public void setStartingPose(Pose pose) {
        if (follower != null) {
            follower.setStartingPose(pose);
        }
    }

    public void setMaxPower(double power){
        if (follower != null) {
            follower.setMaxPower(power);
        }
    }

    /**
     * Returns a PathBuilder pre-bound to this Follower.
     * Use it to build PathChains without touching the Follower directly.
     * Returns null if the Follower is unavailable.
     */
    public PathBuilder pathBuilder() {
        return follower != null ? follower.pathBuilder() : null;
    }

    /**
     * Uses Pedro's drive-only path mode, matching the library's straight-line drive tuner.
     * This leaves translational path following enabled while avoiding heading-hold corrections.
     */
    public void enableDriveOnlyPathMode() {
        if (follower != null) {
            follower.deactivateAllPIDFs();
            follower.activateDrive();
        }
    }

    /**
     * Starts following a pre-built PathChain.
     *
     * @param pathChain the chain to follow, built via {@link #pathBuilder()}.
     * @param holdEnd   if true, the robot will lock onto the final pose when the path completes.
     */
    public void followPath(PathChain pathChain, boolean holdEnd) {
        if (follower != null) {
            follower.followPath(pathChain, holdEnd);
        }
    }

    /**
     * Returns true while the Follower is actively executing a PathChain or holding its end pose.
     * Poll this in the autonomous loop to know when the move is complete.
     */
    public boolean isFollowing() {
        return follower != null && follower.isBusy();
    }

    /**
     * Advances the Pedro Pathing control loop by one tick.
     *
     * <p>In TeleOp: call this <em>once at the very top of the loop</em> (right after clearing the
     * bulk cache) so that:
     * <ol>
     *   <li>The Pinpoint/odometry localizer reads fresh encoder data and updates the robot's pose.</li>
     *   <li>Motor powers queued by {@link #setTeleOpDrive} in the <em>previous</em> loop are applied.</li>
     * </ol>
     * Separating update() from setTeleOpDrive() guarantees odometry is always current regardless of
     * which drive-command branch runs (manual, seek, auto-park, etc.).
     *
     * <p>In Autonomous: call once per loop iteration during path following.
     */
    public void update() {
        if (follower != null) {
            follower.update();
        }
    }

    // ── Accessors ────────────────────────────────────────────────────────────────────────────────

    /** Returns the current robot pose from the Pedro Pathing localizer, or null if unavailable. */
    public Pose getPose() {
        return follower != null ? follower.getPose() : null;
    }

    /** Returns true if the Follower initialized successfully and the drive is operational. */
    public boolean isAvailable() {
        return follower != null;
    }

    /**
     * Returns the error message captured during construction if init failed, or null if healthy.
     * Useful for surfacing motor-config mismatches in telemetry.
     */
    public String getInitError() {
        return initError;
    }

    /** Returns measured LF wheel speed in RPM, or 0 when unavailable. */
    public double getLeftFrontRpm() {
        return toRpm(hardware.getLeftFrontMotor());
    }

    /** Returns measured LR wheel speed in RPM, or 0 when unavailable. */
    public double getLeftRearRpm() {
        return toRpm(hardware.getLeftRearMotor());
    }

    /** Returns measured RF wheel speed in RPM, or 0 when unavailable. */
    public double getRightFrontRpm() {
        return toRpm(hardware.getRightFrontMotor());
    }

    /** Returns measured RR wheel speed in RPM, or 0 when unavailable. */
    public double getRightRearRpm() {
        return toRpm(hardware.getRightRearMotor());
    }

    private static double toRpm(DcMotorEx motor) {
        if (motor == null) {
            return 0.0;
        }

        double ticksPerSecond = motor.getVelocity();
        double ticksPerRev = motor.getMotorType().getTicksPerRev();
        if (ticksPerRev <= 0.0) {
            return 0.0;
        }
        return (ticksPerSecond * 60.0) / ticksPerRev;
    }
}
