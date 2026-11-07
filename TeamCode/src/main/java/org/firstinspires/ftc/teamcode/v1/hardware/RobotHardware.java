package org.firstinspires.ftc.teamcode.v1.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

/**
 * RobotHardware — the only class that calls hardwareMap.get().
 * Sets safe startup defaults for all physical devices and manages bulk caching on every REV hub.
 * Contains no behavior logic — initialization and resource management only.
 *
 * All direct hardwareMap device lookups should be centralized here, including drivetrain motors.
 * Services/subsystems consume typed references from this class instead of calling hardwareMap.get().
 */
public class RobotHardware {

    private List<LynxModule> allHubs;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx rightRearMotor;
    private DcMotorEx leftRearMotor;
    private DcMotorEx leftFrontMotor;
    private GoBildaPinpointDriver odo;
    private WebcamName webcam;
    private DcMotorEx pollenLauncher;
    private DcMotorEx nectarLauncher;
    private DcMotorEx intakeMotor;
    private Servo nectargateServo;
    private Servo pollengateServo;
    //private Servo intakeGuideServo;
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

        // Canonical drivetrain mapping lives here so all direct hardwareMap access is centralized.
        rightFrontMotor = tryGetMotor(hardwareMap, RobotHardwareNames.RIGHT_FRONT_MOTOR);
        rightRearMotor = tryGetMotor(hardwareMap, RobotHardwareNames.RIGHT_REAR_MOTOR);
        leftRearMotor = tryGetMotor(hardwareMap, RobotHardwareNames.LEFT_REAR_MOTOR);
        leftFrontMotor = tryGetMotor(hardwareMap, RobotHardwareNames.LEFT_FRONT_MOTOR);

        applyMotorDefaults(rightFrontMotor);
        applyMotorDefaults(rightRearMotor);
        applyMotorDefaults(leftRearMotor);
        applyMotorDefaults(leftFrontMotor);

        // Optional localization hardware: must not block startup when missing.
        try {
            odo = hardwareMap.get(GoBildaPinpointDriver.class, RobotHardwareNames.PINPOINT);
        } catch (Exception ignored) {
            odo = null;
        }

        // Optional vision hardware: Logitech webcam on USB.
        /*
        try {
            webcam = hardwareMap.get(WebcamName.class, RobotHardwareNames.WEBCAM_NAME);
        } catch (Exception ignored) {
            webcam = null;
        }
         */

        // ── Season devices ───────────────────────────────────────────────────────────────────────
        // Add hardwareMap.get() calls here as new mechanisms are added each season.
        // Example:
        //   liftMotor = hardwareMap.get(DcMotorEx.class, RobotHardwareNames.LIFT_MOTOR);
        //   liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //if intake is not null let power go down slowly when stopped

        pollenLauncher = hardwareMap.get(DcMotorEx.class, RobotHardwareNames.POLLEN_LAUNCHER);
        pollenLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        nectarLauncher = hardwareMap.get(DcMotorEx.class, RobotHardwareNames.NECTAR_LAUNCHER);
        nectarLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeMotor = hardwareMap.get(DcMotorEx.class, RobotHardwareNames.INTAKE_MOTOR);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pollengateServo = hardwareMap.get(Servo.class, RobotHardwareNames.POLLEN_GATE_SERVO);

        nectargateServo = hardwareMap.get(Servo.class, RobotHardwareNames.NECTAR_GATE_SERVO);

        //intakeGuideServo = hardwareMap.get(Servo.class, RobotHardwareNames.INTAKE_GUIDE_SERVO);
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

    private static DcMotorEx tryGetMotor(HardwareMap hardwareMap, String name) {
        try {
            return hardwareMap.get(DcMotorEx.class, name);
        } catch (Exception ignored) {
            return null;
        }
    }

    private static void applyMotorDefaults(DcMotorEx motor) {
        if (motor == null) {
            return;
        }
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public DcMotorEx getRightFrontMotor() {
        return rightFrontMotor;
    }
    public DcMotorEx getRightRearMotor() {
        return rightRearMotor;
    }
    public DcMotorEx getLeftRearMotor() {
        return leftRearMotor;
    }
    public DcMotorEx getLeftFrontMotor() {
        return leftFrontMotor;
    }
    public GoBildaPinpointDriver getPinpoint() {
        return odo;
    }
    public WebcamName getWebcam() {
        return webcam;
    }
    public DcMotorEx getPollenLauncher() {
        return pollenLauncher;
    }
    public DcMotorEx getNectarLauncher() {
        return nectarLauncher;
    }
    public DcMotorEx getIntakeMotor() {
        return intakeMotor;
    }
    public Servo getNectargateServo() {
        return nectargateServo;
    }
    public Servo getPollengateServo() {
        return pollengateServo;
    }
    /*public Servo getIntakeGuideServo() {
        return intakeGuideServo;
    }

     */
}
