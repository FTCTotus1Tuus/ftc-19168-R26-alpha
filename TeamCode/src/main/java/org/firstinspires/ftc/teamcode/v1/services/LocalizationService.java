package org.firstinspires.ftc.teamcode.v1.services;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * LocalizationService — owns robot localization lifecycle.
 *
 * Planned responsibilities:
 * - initialize and validate Pinpoint / odometry hardware
 * - seed follower pose from persisted or sensor pose
 * - provide safe fallback behavior if localization hardware is unavailable
 */
public class LocalizationService {

    private static final String TAG = "LocalizationService";

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public LocalizationService(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    /**
     * Initializes localization hardware and state.
     * Current skeleton implementation is intentionally non-blocking.
     */
    public void initialize() {
        try {
            // TODO: wire GoBildaPinpointDriver and follower pose seeding.
        } catch (Throwable t) {
            RobotLog.ee(TAG, t, "Localization init failed; continuing without localization service");
            telemetry.addLine("[WARN] Localization init failed; using fallback behavior.");
        }
    }

    /** Stops localization resources if needed. */
    public void stop() {
        // TODO: close/release localization resources if future implementations require it.
    }
}

