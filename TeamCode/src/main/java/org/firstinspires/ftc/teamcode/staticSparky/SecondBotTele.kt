package org.firstinspires.ftc.teamcode.staticSparky

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.firstinspires.ftc.teamcode.Controllers.shootingGoal
import org.firstinspires.ftc.teamcode.robotConfigs.RobotBase
import org.firstinspires.ftc.teamcode.robotConfigs.SparkyV2Robot
import java.lang.Math.abs

@TeleOp(name = "Second Robot Tele", group = "StaticDischarge")
class SecondBotTele : SparkOpModeBase() {
    // robot
    lateinit var robot: SparkyV2Robot
    private var reverse = false

    // speeds
    private var driveSpeed = 0.85
    private var lastX = false
    private var IntakeOn = false


    override fun runOpMode() {
        initRobot()
        waitForStart()
        startRobot()
        while (opModeIsActive()) {
            loopRobot()
        }
        stopRobot()
    }

    fun initRobot() {
        //initialize and set robot behavior
        robot = SparkyV2Robot(hardwareMap, telemetry) { true }
        robot.localizer.poseEstimate = Pose2d()
//        robot.loadPose()
    }

    fun startRobot() {

    }

    fun loopRobot() {
        robot.localizer.update()

        // get gamepad input
        // moving the joystick up is actually negative, not positive, so use negative to flip it
        val vert = -gamepad1.left_stick_y.toDouble()
        val hori = gamepad1.left_stick_x.toDouble()
        val turn = gamepad1.right_stick_x.toDouble()
        val wobble = -gamepad1.right_stick_y.toDouble()

        // process input
        if (gamepad1.a) {

            IntakeOn = true

        } else if (gamepad1.b) {

            IntakeOn = false
        }

        if (IntakeOn) {
            robot.roller.start(0.5)
            robot.intake.start(-0.7)
        } else {
            robot.roller.start(0.0)
            robot.intake.start(0.0)
        }


        if (gamepad1.x && !lastX) {
            robot.shooter.shoot()
        }

        if (gamepad1.dpad_up) {
            driveSpeed = 0.9
        } else if (gamepad1.dpad_down){
            driveSpeed = 0.3
        }

        if (gamepad1.left_trigger > 0.3) {
            robot.shooter.simpleShootAtTarget(Pose2d(0.0, 0.0, 0.0), shootingGoal(70.0, 0.0, 35.0))
//            robot.shooter.simpleShootAtTarget(robot.localizer.poseEstimate, Positions.highGoalRed)
        } else {
            robot.shooter.stopWheel()
        }

        robot.arm.run(wobble)
        lastX = gamepad1.x




        try {
//            //output values for robot movement
            robot.driveTrain.start(DriveTrain.Vector(
                    hori * driveSpeed * (if (reverse) -1 else 1).toDouble(),
                    vert * driveSpeed * (if (reverse) -1 else 1).toDouble(),
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

    fun stopRobot() {
        robot.driveTrain.start(DriveTrain.Vector(0.0, 0.0, 0.0).speeds())
    }
}