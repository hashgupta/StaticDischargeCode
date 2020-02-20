package org.firstinspires.ftc.teamcode.FujiCode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FujiCode.Fuji;
import org.firstinspires.ftc.teamcode.FujiCode.FujiAutonomousBase;

@Autonomous(name="RoadRunnerWithBot")
public class FujiAutonomousRunner extends FujiAutonomousBase {

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Fuji(hardwareMap, telemetry, this);

        waitForStart();

        Trajectory test = robot.RoadRunnerDT.trajectoryBuilder()
                .strafeRight(TILE_LENGTH)
                .back(TILE_LENGTH)
                .strafeLeft(TILE_LENGTH)
                .forward(TILE_LENGTH)
                .build();

        robot.RoadRunnerDT.followTrajectorySync(test);
    }
}
