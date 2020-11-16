package org.firstinspires.ftc.teamcode.staticSparky

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "SparkyTests", group = "StaticDischarge")
class SparkAutoTests : SparkAutoBase() {

    override fun runOpMode() {
        // UNCOMMENT THIS IF SOUNDS ARE NEEDED
        val test_robot = SparkyRobot(hardwareMap, telemetry, { opModeIsActive() })
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
        sleep(2000)
        test_robot.telemetry.addLine("moved forward a tile")
        telemetry.update()
        test_robot.turnTo(90.0)
        test_robot.telemetry.addLine("moved to 90 degrees")
        telemetry.update()
        sleep(2000)
        test_robot.turnTo(270.0)
        test_robot.telemetry.addLine("moved to 270 degrees")
        telemetry.update()
        sleep(2000)
        // first build a sequential list of commands
        // then, program out your instructions one by one

    }
}
