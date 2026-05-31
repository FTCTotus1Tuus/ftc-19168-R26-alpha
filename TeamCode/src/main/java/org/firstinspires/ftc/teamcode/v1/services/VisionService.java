package org.firstinspires.ftc.teamcode.v1.services;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.v1.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.v1.hardware.RobotHardwareNames;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

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

    private final RobotHardware hardware;
    private final Telemetry telemetry;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private String initError;

    public VisionService(RobotHardware hardware, Telemetry telemetry) {
        this.hardware = hardware;
        this.telemetry = telemetry;
    }

    /**
     * Initializes vision stack in a fail-safe way.
     */
    public void initialize() {
        initError = null;
        try {
            aprilTagProcessor = new AprilTagProcessor.Builder().build();

            VisionPortal.Builder builder = new VisionPortal.Builder()
                    .addProcessor(aprilTagProcessor);

            // Team standard: Logitech webcam only (no phone/device camera fallback).
            WebcamName webcam = hardware.getWebcam();
            if (webcam == null) {
                initError = "Missing webcam: " + RobotHardwareNames.WEBCAM_NAME;
                telemetry.addLine("[WARN] Vision webcam missing; continuing without camera.");
                aprilTagProcessor = null;
                return;
            }
            builder.setCamera(webcam);

            visionPortal = builder.build();
        } catch (Throwable t) {
            RobotLog.ee(TAG, t, "Vision init failed; continuing without vision");
            telemetry.addLine("[WARN] Vision init failed; check webcam name/config and USB connection.");
            initError = t.getClass().getSimpleName() + ": " + t.getMessage();
            aprilTagProcessor = null;
            visionPortal = null;
        }
    }

    public boolean isAvailable() {
        return visionPortal != null && aprilTagProcessor != null;
    }

    public String getInitError() {
        return initError;
    }

    public List<AprilTagDetection> getAprilTagDetections() {
        if (aprilTagProcessor == null) {
            return Collections.emptyList();
        }
        return Collections.unmodifiableList(new ArrayList<>(aprilTagProcessor.getDetections()));
    }

    /** Stops and releases vision resources. */
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }

        // Keep this placeholder for future processor-specific teardown work.
        aprilTagProcessor = null;
    }
}

