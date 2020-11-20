package org.firstinspires.ftc.teamcode.staticSparky

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "SparkyTests", group = "StaticDischarge")
class SparkAutoTests : SparkAutoBase() {

    override fun runOpMode() {
        // UNCOMMENT THIS IF SOUNDS ARE NEEDED
        robot = SparkyRobot(hardwareMap, telemetry) { opModeIsActive() && !isStopRequested() }
        waitForStart()
        if (isStopRequested()) return

//        test_robot.pursuiter
//                .addPoint(10.0, 10.0, 3.14)
//                .addPoint(5.0, 5.0, 0.0)
//                .addAction { print("hi") }
//                .addTurn(PI)
//                .addPoint(0.0, 0.0, 0.0)
//        test_robot.pursuiter.FollowSync(test_robot.driveTrain)
        robot.move(0.0, TILE_LENGTH)
        telemetry.addLine("moved forward a tile")
        telemetry.update()
        sleep(2000)
        robot.turnTo(90.0)
        telemetry.addLine("moved to 90 degrees")
        telemetry.update()
        sleep(2000)
        robot.turnTo(270.0)
        telemetry.addLine("moved to 270 degrees")
        telemetry.update()
        sleep(2000)
        // first build a sequential list of commands
        // then, program out your instructions one by one

    }
}
