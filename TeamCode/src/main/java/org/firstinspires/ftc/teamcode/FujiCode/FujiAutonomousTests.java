package org.firstinspires.ftc.teamcode.FujiCode;

//@Autonomous(name="FujiTests", group="PatentPending")
public final class FujiAutonomousTests extends FujiAutonomousBase {

    @Override
    public final void runOpMode() {
        // UNCOMMENT THIS IF SOUNDS ARE NEEDED
        robot = new Fuji(hardwareMap, telemetry, this);
        waitForStart();

//        square linear movement
        robot.move(TILE_LENGTH, 0);
        sleep(1000);
        robot.move(0, -TILE_LENGTH);
        sleep(1000);
        robot.move(-TILE_LENGTH, 0);
        sleep(1000);
        robot.move(0, TILE_LENGTH);
        sleep(1000);

//        // turns, gyro sensor
//        while (opModeIsActive()) {
//            telemetry.addData("gyro reading", robot.gyro.measure());
//            telemetry.update();
//        }


    }
}
