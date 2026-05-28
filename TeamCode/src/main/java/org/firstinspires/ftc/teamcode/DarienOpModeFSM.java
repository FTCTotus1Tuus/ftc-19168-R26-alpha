package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
public abstract class DarienOpModeFSM extends LinearOpMode {

    @Override
    public abstract void runOpMode() throws InterruptedException;

    public boolean initControls() {
        return true;
    }
}
