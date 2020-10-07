package org.firstinspires.ftc.teamcode.StaticSparky

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "SparkyTests", group = "StaticDischarge")
class SparkAutoTests : SparkAutoBase() {

    override fun runOpMode() {
        // UNCOMMENT THIS IF SOUNDS ARE NEEDED
        val test_robot = TestRobot(hardwareMap, telemetry)
        waitForStart()
//        while (opModeIsActive()) {
//            if (robot.touch.measure()) {
//                robot.move(0.0, -10.0)
//                robot.turnTo(360*(robot.gyro.measure() + 0.25))
//            }
//            robot.driveTrain.start(DriveTrain.Vector(0.0, 1.0, 0.0).speeds())
//        }

//        test_robot.pursuiter
//                .addPoint(10.0, 10.0, 3.14)
//                .addPoint(5.0, 5.0, 0.0)
//                .addAction { print("hi") }
//                .addTurn(PI)
//                .addPoint(0.0, 0.0, 0.0)
//        test_robot.pursuiter.FollowSync(test_robot.driveTrain)
        test_robot.move(0.0, TILE_LENGTH)
        test_robot.turnTo(90.0)
        // first build a sequential list of commands
        // then, program out your instructions one by one

    }
}
