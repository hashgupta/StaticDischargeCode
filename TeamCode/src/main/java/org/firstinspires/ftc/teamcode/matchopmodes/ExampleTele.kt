package org.firstinspires.ftc.teamcode.matchopmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.controllers.MecanumDriveTrain
import org.firstinspires.ftc.teamcode.robotconfigs.TestRobot


@Disabled
@TeleOp(name = "Example TeleOp", group = "StaticDischarge")
class ExtraTele : OpMode() {
    // robot
    private lateinit var robot: TestRobot

    // speeds
    private var driveSpeed = 1.0

    override fun init() {
        //initialize and set robot behavior
        robot = TestRobot(hardwareMap, telemetry) { true }
        robot.localizer.poseEstimate = Pose2d()
        robot.driveTrain.setZeroBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
        stop()
    }

    override fun loop() {
        robot.localizer.update()

        // get gamepad input
        val vert = gamepad1.left_stick_y.toDouble()
        val hori = gamepad1.left_stick_x.toDouble()
        val turn = gamepad1.right_stick_x.toDouble()


        try {
//            //output values for robot movement
            robot.driveTrain.start(MecanumDriveTrain.Vector(
                    hori * driveSpeed,
                    vert * driveSpeed,
                    turn * driveSpeed)
                    .speeds())

            telemetry.addData("Pose Estimate", robot.localizer.poseEstimate)
            telemetry.update()


        } catch (e: Exception) {
            telemetry.addData("Error", e.message)
            telemetry.addData("info", e.stackTrace[0].toString())
            telemetry.update()
        }

    }


    override fun start() {}

    override fun stop() {
        robot.driveTrain.start(MecanumDriveTrain.Vector(0.0, 0.0, 0.0).speeds())
    }
}