package org.firstinspires.ftc.teamcode.old_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "FujiFoundation", group = "PatentPending")
public class FujiAutonomousFoundationBlue extends FujiAutonomous {

    @Override
    public void runOpMode() {
//        robot = new Fuji(hardwareMap, telemetry, this);
        robot.dropStone.start(0.5);
        waitForStart();

//        robot.drive(0, -0.3, 2, 0.5, false); // drive upto the stone line with a dist of 5 away
//        robot.drive(0.3, 0, 7, 0.5, true); // drive sideways until the the distance is greater than 7

        robot.move(-FOUNDATION_LENGTH_INCH / 2, TILE_LENGTH * 2.1 - ROBOT_EDGE_LENGTH);
//        robot.turn(0.50);
//        robot.hook(1);  //lower servo holders
        sleep(500);

        //move it to the correct position with gyro/distance
        robot.move(0, -TILE_LENGTH * 1.5);
//        robot.hook(0);
        sleep(500);
        robot.move(FOUNDATION_LENGTH_INCH * 0.5, 0.0);
        robot.move(0.0, TILE_LENGTH * 0.1);
        robot.move(TILE_LENGTH * 0.5, 0.0);
        robot.move(0.0, -TILE_LENGTH * 0.1);
        robot.move(TILE_LENGTH * 1.0, 0.0);
    }
}
