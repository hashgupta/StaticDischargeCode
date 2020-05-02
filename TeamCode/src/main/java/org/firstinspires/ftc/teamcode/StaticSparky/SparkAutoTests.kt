package org.firstinspires.ftc.teamcode.StaticSparky

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "SparkyTests", group = "StaticDischarge")
class SparkAutoTests : SparkAutoBase() {

    override fun runOpMode() {
        // UNCOMMENT THIS IF SOUNDS ARE NEEDED
        robot = SparkyRobot(hardwareMap, telemetry)
        waitForStart()
        robot.pursuiter
                .addPoint(10.0, 10.0, 3.14)
                .addPoint(5.0, 5.0, 0.0)
        robot.pursuiter.followSync(robot.driveTrain)

    }
}