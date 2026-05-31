package org.firstinspires.ftc.teamcode.v1.services;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.v1.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.v1.subsystems.DriveSubsystem;

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

    private final RobotHardware hardware;
    private final DriveSubsystem drive;
    private final PreferencesService preferences;
    private final Telemetry telemetry;
    private boolean usingPinpoint;
    private String status = "Not initialized";

    public LocalizationService(RobotHardware hardware,
                               DriveSubsystem drive,
                               PreferencesService preferences,
                               Telemetry telemetry) {
        this.hardware = hardware;
        this.drive = drive;
        this.preferences = preferences;
        this.telemetry = telemetry;
    }

    /**
     * Initializes localization hardware and seeds the follower pose.
     * This method is intentionally fail-safe; startup continues even if Pinpoint is unavailable.
     */
    public void initialize() {
        usingPinpoint = false;
        try {
            GoBildaPinpointDriver pinpoint = hardware.getPinpoint();
            if (pinpoint == null) {
                telemetry.addLine("[WARN] Pinpoint not found; using follower fallback localization.");
                Pose savedPose = preferences.getSavedPose();
                if (savedPose != null && drive.isAvailable()) {
                    drive.setStartingPose(savedPose);
                    status = "Seeded from saved pose (Pinpoint missing)";
                } else {
                    status = "Fallback (no Pinpoint, no saved pose)";
                }
                return;
            }

            pinpoint.resetPosAndIMU();
            pinpoint.update();

            Pose savedPose = preferences.getSavedPose();
            if (drive.isAvailable()) {
                if (savedPose != null) {
                    drive.setStartingPose(savedPose);
                    status = "Seeded from saved pose";
                } else {
                    Pose2D pinpointPose = pinpoint.getPosition();
                    Pose seededPose = new Pose(
                            pinpointPose.getX(DistanceUnit.INCH),
                            pinpointPose.getY(DistanceUnit.INCH),
                            pinpointPose.getHeading(AngleUnit.RADIANS)
                    );
                    drive.setStartingPose(seededPose);
                    status = "Seeded from Pinpoint";
                }
            } else {
                status = "Localization ready (drive unavailable)";
            }

            usingPinpoint = true;
        } catch (Throwable t) {
            RobotLog.ee(TAG, t, "Localization init failed; continuing without localization service");
            telemetry.addLine("[WARN] Localization init failed; using fallback behavior.");
            status = "Fallback after init error";
        }
    }

    public boolean isUsingPinpoint() {
        return usingPinpoint;
    }

    public String getStatus() {
        return status;
    }

    /** Stops localization resources if needed. */
    public void stop() {
        // TODO: add pinpoint stream/telemetry shutdown here if future SDK versions require it.
    }
}

