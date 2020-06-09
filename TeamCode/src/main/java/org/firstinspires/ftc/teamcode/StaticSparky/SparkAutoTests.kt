package org.firstinspires.ftc.teamcode.StaticSparky

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlin.math.PI

@Autonomous(name = "SparkyTests", group = "StaticDischarge")
class SparkAutoTests : SparkAutoBase() {

    override fun runOpMode() {
        // UNCOMMENT THIS IF SOUNDS ARE NEEDED
        robot = SparkyRobot(hardwareMap, telemetry)
        waitForStart()
        robot.pursuiter
                .addPoint(10.0, 10.0, 3.14)
                .addPoint(5.0, 5.0, 0.0)
                .addAction { print("hi") }
                .addTurn(PI)
                .addPoint(0.0, 0.0, 0.0)
        robot.pursuiter.FollowSync(robot.driveTrain)

    }
}
