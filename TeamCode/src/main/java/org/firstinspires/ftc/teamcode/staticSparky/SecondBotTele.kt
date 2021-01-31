package org.firstinspires.ftc.teamcode.staticSparky

import com.acmerobotics.roadrunner.drive.Drive
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.firstinspires.ftc.teamcode.Controllers.shootingGoal
import org.firstinspires.ftc.teamcode.Positions
import org.firstinspires.ftc.teamcode.robotConfigs.RobotBase
import org.firstinspires.ftc.teamcode.robotConfigs.SparkyV2Robot
import java.lang.Math.abs

@TeleOp(name = "Second Robot Tele", group = "StaticDischarge")
class SecondBotTele : SparkOpModeBase() {
    // robot
    lateinit var robot: SparkyV2Robot

    // speeds
    private var driveSpeed = 0.95
    private var lastTriggerRight = 0.0
    private var IntakeOn = false
    private var previousGamepad1Guide = false


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
        robot.loadPose()
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
        val wobble = -gamepad2.right_stick_y.toDouble()

        // process input
        if (gamepad1.a) {

            IntakeOn = true

        } else if (gamepad1.b) {

            IntakeOn = false
        }

        if (IntakeOn) {
            robot.roller.start(0.7)
            robot.intake.start(-0.35)
        } else {
            robot.roller.start(0.0)
            robot.intake.start(0.0)
        }


        if (gamepad2.right_trigger  > lastTriggerRight) {
            robot.shooter.shoot()
        }

        if (gamepad1.dpad_up) {
            driveSpeed = 0.9
        } else if (gamepad1.dpad_down){
            driveSpeed = 0.3
        }

        if (gamepad2.left_trigger > 0.3) {
            robot.shooter.simpleShootAtTarget(Pose2d(0.0, 0.0, 0.0), shootingGoal(70.0, 0.0, 36.0))
//            robot.shooter.simpleShootAtTarget(robot.localizer.poseEstimate, Positions.highGoalRed)
        } else {
            robot.shooter.stopWheel()
        }

        if (gamepad1.guide && !previousGamepad1Guide) {
//            if (timer.seconds() < 90.0) {
//                robot.pursuiter.setStartPoint(robot.localizer.poseEstimate)
//                robot.pursuiter.addTurnAbsolute(
//                        robot.shooter.turningTarget(robot.localizer.poseEstimate.vec(), Positions.highGoalRed))
//
//                robot.pursuiter.FollowSync(robot.driveTrain, telemetry = telemetry)
//            } else {
//                //turn towards power shots
//                robot.pursuiter.setStartPoint(robot.localizer.poseEstimate)
//                robot.pursuiter.addTurnAbsolute(
//                        robot.shooter.turningTarget(robot.localizer.poseEstimate.vec(), Positions.powerNearRed))
//
//                robot.pursuiter.FollowSync(robot.driveTrain, telemetry = telemetry)
//            }
            robot.pursuiter.setStartPoint(robot.localizer.poseEstimate)
            robot.pursuiter.addTurnAbsolute(
                    robot.shooter.turningTarget(robot.localizer.poseEstimate.vec(), Positions.highGoalRed))

            robot.pursuiter.FollowSync(robot.driveTrain, telemetry = telemetry)
        }

        robot.arm.run(wobble)
        lastTriggerRight = gamepad2.right_trigger.toDouble()
        previousGamepad1Guide = gamepad1.guide




        try {
            //output values for robot movement
            var wheelSpeeds = DriveTrain.Vector(hori, vert, turn).speeds()
            wheelSpeeds = DriveTrain.multiplySquare(speeds = wheelSpeeds, scalar = driveSpeed)
            robot.driveTrain.start(wheelSpeeds)
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