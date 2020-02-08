package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

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
