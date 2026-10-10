package org.firstinspires.ftc.teamcode.v1.services;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotorHelper {
    private double pduty, power, iduty, clampedsetpoint, ticksPerRotation;
    private final Telemetry telemetry;

    public MotorHelper(Telemetry telemetry, double ticksPerRotation) {
        this.pduty = 0;
        this.power = 0;
        this.iduty = 0;
        this.clampedsetpoint = 0;
        this.telemetry = telemetry;
        this.ticksPerRotation = ticksPerRotation;
    }

    /**
     * pid
     * Proportional-integral controller for a DcMotor.
     * Returns the power required to maintain the motor encoder at a desired set point.
     * Written by Vivan, Mario, and Aiden
     */
    public double[] pidPositional(DcMotor Motor, double pgain, double pgain2, double igain, double setpoint, double pdutyMin, double pdutyMax, double idutyMin, double idutyMax, double iduty, double powerMin, double powerMax, double gain, double bottomStop, double topStop, float Control, boolean isTelemetryShown) {
        double[] returnArray;

        //If the motor is going up
        if (Control <= 0) {
            this.clampedsetpoint = clamp(setpoint + Control * gain, bottomStop, topStop);

            this.pduty = clamp(pgain * (this.clampedsetpoint - Motor.getCurrentPosition()), pdutyMin, pdutyMax);
            this.iduty = clamp(igain * (this.clampedsetpoint - Motor.getCurrentPosition()) + iduty, idutyMin, idutyMax);
            this.power = clamp(this.pduty + this.iduty, powerMin, powerMax);
        }

        //If the motor is going down
        if (Control > 0) {
            this.clampedsetpoint = clamp(setpoint + Control * gain, bottomStop, topStop);

            this.pduty = clamp(pgain2 * (this.clampedsetpoint - Motor.getCurrentPosition()), pdutyMin, pdutyMax);
            this.iduty = clamp(igain * (this.clampedsetpoint - Motor.getCurrentPosition()) + iduty, idutyMin, idutyMax);
            this.power = clamp(this.pduty + this.iduty, powerMin, powerMax);
        }


        // kills motor when it should be at rest
        if (setpoint <= bottomStop + 10 && Motor.getCurrentPosition() <= bottomStop + 10) {
            Motor.setPower(0);
        }

        if (isTelemetryShown) {
            telemetry.addData("Pduty: ", pduty);
            telemetry.addData("Iduty: ", iduty);
            telemetry.addData("Motor Encoder " + Motor.getDeviceName() + ": ", Motor.getCurrentPosition());
            telemetry.addData("Set point: ", setpoint);
            telemetry.addData("Clamped Set Point: ", clampedsetpoint);
        }

        returnArray = new double[]{this.power, this.iduty, this.clampedsetpoint};
        return returnArray;
    }

    public static double[] pidRunWithEncoder(DcMotorEx motor, double pgain, double pgain2, double igain, double setpoint, double pdutyMin, double pdutyMax, double idutyMin, double idutyMax, double iduty, double powerMin, double powerMax, double gain, double minRPM, double maxRPM, float control, boolean isTelemetryShown) {
        double clampedSetpoint = clamp(setpoint + control * gain, minRPM, maxRPM);

        double ticksPerRotation = motor.getMotorType().getTicksPerRev();
        double currentRpm = getRpmFromTicksPerSecond(Math.abs(motor.getVelocity()), ticksPerRotation);

        double usedPgain = control > 0 ? pgain2 : pgain;
        double error = clampedSetpoint - currentRpm;

        double pdutyOut = clamp(usedPgain * error, pdutyMin, pdutyMax);
        double idutyOut = clamp(iduty + (igain * error), idutyMin, idutyMax);
        double powerOut = clamp(pdutyOut + idutyOut, powerMin, powerMax);

        return new double[]{powerOut, idutyOut, clampedSetpoint, pdutyOut};
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    private static double getRpmFromTicksPerSecond(double ticksPerSecond, double ticksPerRotation) {
        if (ticksPerRotation <= 0) return ticksPerSecond;
        return (ticksPerSecond * 60.0) / ticksPerRotation;
    }

    public double getRpmFromTicksPerSecond(double ticksPerSecond) {
        return getRpmFromTicksPerSecond(ticksPerSecond, ticksPerRotation);
    }
}
