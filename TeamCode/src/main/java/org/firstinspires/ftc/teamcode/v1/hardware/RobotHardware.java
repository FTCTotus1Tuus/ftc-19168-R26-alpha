package org.firstinspires.ftc.teamcode.v1.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * RobotHardware — the only class that calls hardwareMap.get().
 * Sets safe startup defaults for all physical devices and manages bulk caching on every REV hub.
 * Contains no behavior logic — initialization and resource management only.
 *
 * Drive motors are an accepted exception: they are mapped internally by PedroPathingConstants
 * when DriveSubsystem constructs its Follower. All other devices (lift, intake, sensors, etc.)
 * must be mapped here.
 */
public class RobotHardware {

    private List<LynxModule> allHubs;

    /**
     * Maps all hardware devices and enables manual bulk caching on every REV hub.
     * Must be called first inside RobotContainer.initialize(), before any subsystem init.
     */
    public void initialize(HardwareMap hardwareMap) {
        // ── Bulk caching ────────────────────────────────────────────────────────────────────────
        // MANUAL mode: one bulk read per hub per call to clearBulkCache().
        // clearBulkCache() must be the very first call in every OpMode loop.
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // ── Season devices ───────────────────────────────────────────────────────────────────────
        // Add hardwareMap.get() calls here as new mechanisms are added each season.
        // Example:
        //   liftMotor = hardwareMap.get(DcMotorEx.class, RobotHardwareNames.LIFT_MOTOR);
        //   liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Clears the bulk-read cache on every REV hub.
     * Must be the very first call at the top of each OpMode loop so all reads in that
     * loop iteration receive fresh data.
     */
    public void clearBulkCache() {
        if (allHubs == null) return;
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    /**
     * Releases resources held by the hardware layer.
     * Add device.close() calls here for sensors that require explicit cleanup.
     */
    public void stop() {
        // Nothing to release in the base skeleton.
    }
}


