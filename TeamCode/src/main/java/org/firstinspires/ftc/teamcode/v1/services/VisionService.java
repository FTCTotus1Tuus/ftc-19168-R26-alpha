package org.firstinspires.ftc.teamcode.v1.services;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.v1.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.v1.hardware.RobotHardwareNames;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.teamcode.v1.config.VisionConfig;

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
    private ColorBlobLocatorProcessor ballProcessor;
    private String initError;

    public VisionService(RobotHardware hardware, Telemetry telemetry) {
        this.hardware = hardware;
        this.telemetry = telemetry;
    }

    /**
     * Initializes vision stack in a fail-safe way.
     * Both processors share one VisionPortal — a webcam can only be opened once.
     */
    public void initialize() {
        initError = null;
        try {
            // Team standard: Logitech webcam only (no phone/device camera fallback).
            WebcamName webcam = hardware.getWebcam();
            if (webcam == null) {
                initError = "Missing webcam: " + RobotHardwareNames.WEBCAM_NAME;
                telemetry.addLine("[WARN] Vision webcam missing; continuing without camera.");
                return;
            }

            // Build both processors up front so they can share one portal.
            aprilTagProcessor = new AprilTagProcessor.Builder().build();

            ballProcessor = new ColorBlobLocatorProcessor.Builder()
                    .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)

                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                    .setRoi(ImageRegion.entireFrame())
                    .setDrawContours(true)
                    .setBoxFitColor(0)       // Disable the drawing of rectangles
                    .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                    .setBlurSize(5)
                    .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                    .setErodeSize(15)        // Shrink blobs back to original size
                    .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                    .build();

            // One portal, one camera open — add every processor here.
            visionPortal = new VisionPortal.Builder()
                    .setCamera(webcam)
                    .addProcessor(aprilTagProcessor)
                    .addProcessor(ballProcessor)
                    .setCameraResolution(new Size(320, 240))
                    // Lower res for faster processing;
                    // Lower res does not require high light conditions to detect balls (Willy)

                    .build();

        } catch (Throwable t) {
            RobotLog.ee(TAG, t, "Vision init failed; continuing without vision");
            telemetry.addLine("[WARN] Vision init failed; check webcam name/config and USB connection.");
            initError = t.getClass().getSimpleName() + ": " + t.getMessage();
            aprilTagProcessor = null;
            ballProcessor = null;
            visionPortal = null;
        }
    }

    /** Portal is up and ball processor is ready. AprilTag is a bonus — don't gate on it. */
    public boolean isAvailable() {
        return visionPortal != null && ballProcessor != null;
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

    public static final class BallTarget {
        public final boolean isVisible;
        public final double normalizedXError; // -1..1, right is positive
        public final double radiusPx;
        public final double circularity;
        public final double circleFitX;
        public final double circleFitY;

        public BallTarget(boolean isVisible, double normalizedXError, double radiusPx, double circularity, double circleFitX, double circleFitY) {
            this.isVisible = isVisible;
            this.normalizedXError = normalizedXError;
            this.radiusPx = radiusPx;
            this.circularity = circularity;
            this.circleFitX = circleFitX;
            this.circleFitY = circleFitY;
        }

        public static BallTarget notVisible() {
            return new BallTarget(false, 0.0, 0.0, 0.0, 0.0, 0.0);
        }
    }

    public BallTarget getBallTarget() {
        if (ballProcessor == null) {
            return BallTarget.notVisible();
        }

        List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>(ballProcessor.getBlobs());

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                VisionConfig.BALL_MIN_CONTOUR_AREA_PX,
                VisionConfig.BALL_MAX_CONTOUR_AREA_PX,
                blobs
        );

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                VisionConfig.BALL_MIN_CIRCULARITY,
                1.0,
                blobs
        );

        if (blobs.isEmpty()) {
            return BallTarget.notVisible();
        }

        ColorBlobLocatorProcessor.Blob best = blobs.get(0);
        Circle circle = best.getCircle();

        double frameCenterX = VisionConfig.BALL_CAMERA_WIDTH_PX * 0.5;
        double normalizedXError = (circle.getX() - frameCenterX) / frameCenterX;

        return new BallTarget(true, normalizedXError, circle.getRadius(), best.getCircularity(), circle.getX(), circle.getY());
    }

    /** Stops and releases vision resources. */
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }

        // Keep this placeholder for future processor-specific teardown work.
        aprilTagProcessor = null;
        ballProcessor = null;
    }
}
