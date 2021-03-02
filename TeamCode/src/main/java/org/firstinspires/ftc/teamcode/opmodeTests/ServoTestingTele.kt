package org.firstinspires.ftc.teamcode.opmodeTests

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robotConfigs.SparkyV2Robot
import java.lang.Math.abs

@TeleOp(name = "Servo Testing TeleOP", group = "Static Discharge")
class IntakeTestingTele: OpMode() {
    lateinit var robot: SparkyV2Robot

    override fun init() {
        robot = SparkyV2Robot(hardwareMap, telemetry) { false }

    }

    override fun loop() {

        robot.arm.grabber?.start(abs(gamepad1.right_stick_y.toDouble()))
        robot.shooter.flicker!!.start(abs(gamepad1.left_stick_y.toDouble()))

        telemetry.addLine(gamepad1.right_stick_y.toDouble().toString())
        telemetry.addLine(abs(gamepad1.left_stick_y.toDouble()).toString())
        telemetry.update()


    }

}