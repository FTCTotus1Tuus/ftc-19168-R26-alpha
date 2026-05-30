package org.firstinspires.ftc.teamcode.v1.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.v1.core.RobotContainer;

/**
 * RobotOpMode — thin base class for all team OpModes.
 * Owns the RobotContainer lifecycle. Subclasses implement runOpMode().
 *
 * Must remain small and stable across seasons.
 * Do NOT add season-specific logic, mechanism control, or hardware references here.
 */
public abstract class RobotOpMode extends LinearOpMode {

    protected RobotContainer robot;

    /**
     * Creates and initializes RobotContainer.
     * Call at the top of runOpMode(), before telemetry.addLine("Ready").
     */
    protected void initRobot() {
        robot = new RobotContainer(hardwareMap, telemetry);
        robot.initialize();
    }

    /**
     * Stops all subsystems cleanly.
     * Call at the end of runOpMode(), after the main loop exits.
     */
    protected void stopRobot() {
        if (robot != null) {
            robot.stop();
        }
    }
}


