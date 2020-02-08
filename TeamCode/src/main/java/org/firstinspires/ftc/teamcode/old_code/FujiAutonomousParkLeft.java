package org.firstinspires.ftc.teamcode.old_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "FujiFoundationParkLeft", group = "PatentPending")
public class FujiAutonomousParkLeft extends FujiAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
//        Fuji robot = new Fuji(hardwareMap, telemetry, this);
        robot.dropStone.start(0.5);
        waitForStart();
        robot.move(TILE_LENGTH, 0);

    }
}
