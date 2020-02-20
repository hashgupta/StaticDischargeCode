package org.firstinspires.ftc.teamcode.FujiCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="CVAUTOTEST")
public class FujiAutonomousCV extends FujiAutonomousBase {

    @Override
    final public void runOpMode() throws InterruptedException {
        initCV();
        waitForStart();
        startCV();
        while (opModeIsActive()) {
            telemetry.addData("average", pipeline.average());
            telemetry.update();
        }
        stopCV();
    }
}