package org.firstinspires.ftc.teamcode;

public class FujiAutonomousCV extends FujiAutonomousBase {

    @Override
    final public void runOpMode() throws InterruptedException {
        initCV();
        waitForStart();
        startCV();
        while (opModeIsActive()) {
            idle();
        }
        stopCV();
    }
}
