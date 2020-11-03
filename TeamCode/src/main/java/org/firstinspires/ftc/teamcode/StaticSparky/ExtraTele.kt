package org.firstinspires.ftc.teamcode.StaticSparky

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.firstinspires.ftc.teamcode.Controllers.shootingGoal
import kotlin.math.abs

@TeleOp(name = "Special Drive Tele", group = "StaticDischarge")
class ExtraTele : OpMode() {
    // robot
    private lateinit var robot: SparkyRobot
    private var reverse = false

    // speeds
    private var driveSpeed = 1.0

    override fun init() {
        //initialize and set robot behavior
        robot = SparkyRobot(hardwareMap, telemetry)
        robot.driveTrain.setZeroBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
        stop()
    }

    override fun loop() {

        // get gamepad input
        var vert = (gamepad1.left_stick_y.toDouble() + gamepad1.right_stick_y.toDouble()) * 0.5
        val turn = (gamepad1.left_stick_y.toDouble() - gamepad1.right_stick_y.toDouble()) * 0.5
        var hori = (gamepad1.left_stick_x.toDouble() + gamepad2.right_stick_x) * 0.5

        // process input


        if (gamepad1.dpad_down) {
            driveSpeed = 0.5
        }
        if (gamepad1.dpad_up) {
            driveSpeed = 1.0
        }

        if (gamepad1.x) {
            reverse = true
        }
        if (gamepad1.y) {
            reverse = false
        }

        if (abs(vert) < 0.1) {
            vert = 0.0
        }
        if (abs(hori) < 0.1) {
            hori = 0.0
        }

        try {
//            //output values for robot movement
            robot.driveTrain.start(DriveTrain.Vector(
                    hori * driveSpeed * (if (reverse) -1 else 1).toDouble(),
                    vert * driveSpeed * (if (reverse) -1 else 1).toDouble(),
                    turn * driveSpeed)
                    .speeds())
//            robot.lift.start(liftSpeed(lift))


        } catch (e: Exception) {
            telemetry.addData("Error", e.message)
            telemetry.addData("info", e.stackTrace[0].toString())
            telemetry.update()
        }

    }


    override fun start() {}

    override fun stop() {
        robot.driveTrain.start(DriveTrain.Vector(0.0, 0.0, 0.0).speeds())
    }

    companion object {
        // field measurements

    }
}