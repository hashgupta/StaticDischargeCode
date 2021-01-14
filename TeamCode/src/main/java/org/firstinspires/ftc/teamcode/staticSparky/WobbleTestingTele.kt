package org.firstinspires.ftc.teamcode.StaticSparky

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robotConfigs.SparkyRobot

@TeleOp(name = "Wobble Testing TeleOP", group = "Static Discharge")
class WobbleTestingTele: LinearOpMode() {
    lateinit var robot: SparkyRobot

    override fun runOpMode() {
        robot = SparkyRobot(hardwareMap, telemetry) { false }
        robot.arm.grabAuto()

        waitForStart()
//        while (opModeIsActive()) {
//            telemetry.addLine(robot.arm.arm_motor.device.currentPosition.toString())
//            telemetry.update()
//            if (gamepad1.a) {
//                robot.arm.dropAuto()
//            }
//            if (gamepad1.b) {
//                robot.arm.grabAuto()
//            }
//        }
    }

}