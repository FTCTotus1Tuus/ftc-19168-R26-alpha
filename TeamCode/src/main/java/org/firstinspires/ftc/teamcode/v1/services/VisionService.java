package org.firstinspires.ftc.teamcode.v1.services;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * VisionService — owns vision lifecycle (camera + processors).
 *
 * Planned responsibilities:
 * - open VisionPortal and processors in initialize()
 * - expose immutable detection snapshots to callers
 * - close camera resources in stop()
 */
public class VisionService {

    private static final String TAG = "VisionService";

    private final Telemetry telemetry;

    public VisionService(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Initializes vision stack.
     * Current skeleton implementation is intentionally non-blocking.
     */
    public void initialize() {
        try {
            // TODO: initialize VisionPortal + processors.
        } catch (Throwable t) {
            RobotLog.ee(TAG, t, "Vision init failed; continuing without vision");
            telemetry.addLine("[WARN] Vision init failed; continuing without camera.");
        }
    }

    /** Stops and releases vision resources. */
    public void stop() {
        // TODO: close VisionPortal when implemented.
    }
}

