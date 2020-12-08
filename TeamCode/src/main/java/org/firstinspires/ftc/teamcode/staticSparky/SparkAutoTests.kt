package org.firstinspires.ftc.teamcode.staticSparky

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlin.math.PI

@Autonomous(name = "SparkyTests", group = "StaticDischarge")
class SparkAutoTests : SparkAutoBase() {

    override fun runOpMode() {
        // UNCOMMENT THIS IF SOUNDS ARE NEEDED
        robot = SparkyRobot(hardwareMap, telemetry) { opModeIsActive() && !isStopRequested() }
        waitForStart()

        robot.pursuiter
//                .addPoint(10.0, 10.0, 1.57)
                .addPoint(50.0, -40.0, 0.0)
                .addAction { telemetry.addLine("hi"); telemetry.update() }
//                .addTurn(PI)
//                .addPoint(0.0, 0.0, 0.0)

        robot.pursuiter.FollowSync(robot.driveTrain, telemetry = telemetry)


    }
}
