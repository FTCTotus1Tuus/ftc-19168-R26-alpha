
package org.firstinspires.ftc.teamcode.v1.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.v1.config.DriveConfig;
import org.firstinspires.ftc.teamcode.v1.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.v1.services.PedroPathingConstants;

public class DriveSubsystem {

    private static final String TAG = "DriveSubsystem";

    private final RobotHardware hardware;
    private Follower follower;
    private String initError = null;

    public DriveSubsystem(HardwareMap hardwareMap, RobotHardware hardware) {
        this.hardware = hardware;

        try {
            follower = PedroPathingConstants.createFollower(hardwareMap);
        } catch (Exception e) {
            RobotLog.ee(TAG, e, "Follower initialization failed");
            initError = e.getClass().getSimpleName() + ": " + e.getMessage();
            follower = null;
        }
    }

    // Lifecycle

    public void initialize() {
        if (follower != null) {
            follower.update();
        }
    }

    public void resumeTeleOpDrive() {
        // Pedro 3 manual drive is commanded through manual()
        // each loop; there is no need to start a separate
        // teleop-drive mode.
    }

    public void stop() {
        if (follower != null) {
            follower.stop();
        }
    }

    // TeleOp

    public void setTeleOpDrive(
            double forward,
            double strafe,
            double turn
    ) {
        if (follower != null) {
            follower.manual(
                    forward,
                    strafe,
                    turn * DriveConfig.TELEOP_ROTATION_SCALE
            );
        }
    }

    // Autonomous

    public void setStartingPose(Pose pose) {
        if (follower != null) {
            follower.setPose(pose);
        }
    }

    public void setPose(Pose pose) {
        if (follower != null) {
            follower.setPose(pose);
        }
    }

    public void followPath(Path path) {
        if (follower != null && path != null) {
            follower.follow(path);
        }
    }

    public boolean isFollowing() {
        return follower != null && follower.isBusy();
    }

    public void update() {
        if (follower != null) {
            follower.update();
        }
    }

    // Accessors

    public Pose getPose() {
        return follower != null ? follower.pose() : null;
    }

    public boolean isAvailable() {
        return follower != null;
    }

    public String getInitError() {
        return initError;
    }

    // Motor telemetry

    public double getLeftFrontRpm() {
        return toRpm(hardware.getLeftFrontMotor());
    }

    public double getLeftRearRpm() {
        return toRpm(hardware.getLeftRearMotor());
    }

    public double getRightFrontRpm() {
        return toRpm(hardware.getRightFrontMotor());
    }

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

        return ticksPerSecond * 60.0 / ticksPerRev;
    }
}
