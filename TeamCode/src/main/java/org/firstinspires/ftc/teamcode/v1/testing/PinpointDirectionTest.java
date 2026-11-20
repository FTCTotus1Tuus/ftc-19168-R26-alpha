
package org.firstinspires.ftc.teamcode.v1.testing;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.v1.services.PedroPathingConstants;

@Autonomous(name = "Pinpoint Direction Test", group = "Testing")
public class PinpointDirectionTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Follower follower =
                PedroPathingConstants.createFollower(hardwareMap);

        follower.setPose(new Pose(0, 0, 0));
        follower.update();

        telemetry.addLine("Pinpoint initialized");
        telemetry.addLine("Place robot on flat floor.");
        telemetry.addLine("Record starting X and Y.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            follower.update();

            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData(
                    "Heading (degrees)",
                    Math.toDegrees(follower.pose().heading())
            );

            telemetry.update();
        }
    }
}
