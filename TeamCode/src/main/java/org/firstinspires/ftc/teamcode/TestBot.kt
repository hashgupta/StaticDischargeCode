package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.firstinspires.ftc.teamcode.hardware.general.Motor

@TeleOp(name = "LilBot", group = "PatentPending")
class TestBot : OpMode() {

    private lateinit var robot: DriveTrain

    override fun init() {
        robot = DriveTrain(
                Motor("rf", 1120.0, hardwareMap),
                Motor("rb", 1120.0, hardwareMap),
                Motor("lf", 1120.0, hardwareMap),
                Motor("lb", 1120.0, hardwareMap))
        stop()
    }

    override fun start() {}

    override fun loop() {
        robot.start(DriveTrain.Vector(
                gamepad1.left_stick_x.toDouble(),
                gamepad1.left_stick_y.toDouble(),
                gamepad1.right_stick_x.toDouble()).speeds())
    }

    override fun stop() {
        robot.start(DriveTrain.Vector(0.0, 0.0, 0.0).speeds())
    }
}