package org.firstinspires.ftc.teamcode.StaticSparky

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.staticSparky.SparkyRobot

@TeleOp(name = "Intake Testing TeleOP", group = "Static Discharge")
class IntakeTestingTele: OpMode() {
    lateinit var robot:SparkyRobot

    override fun init() {
        robot = SparkyRobot(hardwareMap, telemetry) {false}

    }

    override fun loop() {


        robot.arm.grabber.start(gamepad1.right_stick_y.toDouble())
        robot.shooter.flicker!!.start(gamepad1.left_stick_y.toDouble())


    }

}