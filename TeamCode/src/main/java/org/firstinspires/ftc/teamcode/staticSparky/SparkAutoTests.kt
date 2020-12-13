package org.firstinspires.ftc.teamcode.staticSparky

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlin.math.PI

@Autonomous(name = "SparkyTests", group = "StaticDischarge")
class SparkAutoTests : SparkAutoBase() {

    override fun runOpMode() {
        // UNCOMMENT THIS IF SOUNDS ARE NEEDED
        robot = SparkyRobot(hardwareMap, telemetry) { opModeIsActive() && !isStopRequested() }
        waitForStart()


        robot.arm.toAngle(0.0)
        while (robot.arm.arm_motor.isBusy) {
            telemetry.addLine(robot.arm.arm_motor.device.currentPosition.toString())
            telemetry.addLine(robot.arm.arm_motor.device.targetPosition.toString())
            telemetry.update()
        }
        robot.arm.run(0.0)
        sleep(4000)


    }
}
