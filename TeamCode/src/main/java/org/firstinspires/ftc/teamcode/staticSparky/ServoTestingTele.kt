package org.firstinspires.ftc.teamcode.staticSparky

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robotConfigs.SparkyRobot
import java.lang.Math.abs

@TeleOp(name = "Servo Testing TeleOP", group = "Static Discharge")
class IntakeTestingTele: OpMode() {
    lateinit var robot: SparkyRobot

    override fun init() {
        robot = SparkyRobot(hardwareMap, telemetry) { false }

    }

    override fun loop() {

        robot.arm.grabber.start(abs(gamepad1.right_stick_y.toDouble()))
        robot.shooter.flicker!!.start(abs(gamepad1.left_stick_y.toDouble()))

        telemetry.addLine(gamepad1.right_stick_y.toDouble().toString())
        telemetry.addLine(abs(gamepad1.left_stick_y.toDouble()).toString())
        telemetry.update()


    }

}