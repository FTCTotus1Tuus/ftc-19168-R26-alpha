package org.firstinspires.ftc.teamcode.v1.core;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.v1.services.LocalizationService;
import org.firstinspires.ftc.teamcode.v1.services.PedroPathingConstants;
import org.firstinspires.ftc.teamcode.v1.services.PreferencesService;
import org.firstinspires.ftc.teamcode.v1.services.VisionService;
import org.firstinspires.ftc.teamcode.v1.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.v1.subsystems.DriveSubsystem;

/**
 * RobotContainer — single composition root for all subsystems and services.
 * Created and owned by RobotOpMode. Subsystem references are public fields so
 * OpModes can call intent methods (e.g., robot.drive.setTeleOpDrive(...)).
 *
 * Construction order: hardware first, then subsystems, then services.
 * Initialize() order mirrors construction; stop() is the reverse.
 */
public class RobotContainer {

    // ── Public subsystem references ──────────────────────────────────────────────────────────────
    // Access these from OpModes: robot.drive, robot.hardware, etc.

    public final RobotHardware hardware;
    public final DriveSubsystem drive;
    public final LocalizationService localization;
    public final VisionService vision;
    public final PreferencesService preferences;

    // Add season subsystems here as public final fields, e.g.:
    //   public final LiftSubsystem lift;

    // ── Private fields ───────────────────────────────────────────────────────────────────────────

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    // ── Lifecycle ────────────────────────────────────────────────────────────────────────────────

    public RobotContainer(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry   = telemetry;

        // Construct subsystems — no initialization yet; that happens in initialize().
        hardware = new RobotHardware();
        drive    = new DriveSubsystem(hardwareMap);

        localization = new LocalizationService(hardwareMap, telemetry);
        vision       = new VisionService(telemetry);
        preferences  = new PreferencesService();

        // Season subsystems:
        //   lift = new LiftSubsystem(hardwareMap);
    }

    /**
     * Initializes all subsystems in dependency order.
     * RobotHardware must run first to enable bulk caching before any subsystem reads hardware.
     * Any optional subsystem whose init might fail must be wrapped in a try/catch here.
     */
    public void initialize() {
        // 1. Hardware layer (bulk caching setup + direct device mapping)
        hardware.initialize(hardwareMap);

        // 2. Drive
        if (drive.isAvailable()) {
            drive.initialize();
        } else {
            telemetry.addLine("[WARN] Drive unavailable — check motor config.");
            telemetry.addLine("       " + drive.getInitError());
            telemetry.addData("Available motors",
                    PedroPathingConstants.getAvailableMotorNames(hardwareMap).toString());
        }

        // 3. Core services (must not block startup)
        localization.initialize();
        vision.initialize();
        preferences.initialize();

        // 4. Season subsystems:
        //   lift.initialize();
    }

    /**
     * Stops all subsystems in reverse-initialization order.
     * Called by RobotOpMode.stopRobot() at the end of every OpMode.
     */
    public void stop() {
        // Season subsystems first:
        //   lift.stop();

        preferences.stop();
        vision.stop();
        localization.stop();
        drive.stop();
        hardware.stop();
    }
}
