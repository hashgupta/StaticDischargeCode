package org.firstinspires.ftc.teamcode.StaticSparky

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain

@Autonomous(name = "SparkyTests", group = "StaticDischarge")
class SparkAutoTests : SparkAutoBase() {

    override fun runOpMode() {
        // UNCOMMENT THIS IF SOUNDS ARE NEEDED
        robot = SparkyRobot(hardwareMap, telemetry)
        waitForStart()
        while (opModeIsActive()) {
            if (robot.touch.measure()) {
                robot.move(0.0, -10.0)
                robot.turnTo(360*(robot.gyro.measure() + 0.25))
            }
            robot.driveTrain.start(DriveTrain.Vector(0.0, 1.0, 0.0).speeds())
        }

//        robot.pursuiter
//                .addPoint(10.0, 10.0, 3.14)
//                .addPoint(5.0, 5.0, 0.0)
//                .addAction { print("hi") }
//                .addTurn(PI)
//                .addPoint(0.0, 0.0, 0.0)
//        robot.pursuiter.FollowSync(robot.driveTrain)
        // first build a sequential list of commands
        // then, program out your instructions one by one

    }
}
