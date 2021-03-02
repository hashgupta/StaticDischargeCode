package org.firstinspires.ftc.teamcode.staticSparky

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.firstinspires.ftc.teamcode.Controllers.shootingGoal
import org.firstinspires.ftc.teamcode.Positions
import org.firstinspires.ftc.teamcode.robotConfigs.SparkyV2Robot

@TeleOp(name = "Second Robot Tele", group = "StaticDischarge")
class SecondBotTele : SparkOpModeBase() {
    // robot
    lateinit var robot: SparkyV2Robot

    // speeds
    enum class DriveSpeeds {
        Normal,
        Slow
    }

    private var driveSpeed = DriveSpeeds.Normal
    private var lastTriggerRight = 0.0
    private var IntakeOn = false
    private var previousGamepad1X = false
    private var IntakeBackwards = false

    private val normalSpeed = 0.95
    private val slowSpeed = 0.3


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
        val hori = gamepad1.left_stick_x.toDouble() * 1.1
        val turn = gamepad1.right_stick_x.toDouble()
        val wobble = -gamepad2.right_stick_y.toDouble()

        // process input
        if (gamepad1.a) {

            IntakeOn = true

        } else if (gamepad1.b) {

            IntakeOn = false
        }
//
//
//        if (gamepad2.a) {
//
//            robot.arm.arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
//
//        } else if (gamepad2.b) {
//
//            robot.arm.arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
//
//        }

        if (gamepad1.right_bumper) {
            IntakeBackwards = true
        } else if (gamepad1.left_bumper) {
            IntakeBackwards = false
        }


        if (IntakeOn) {
            if (IntakeBackwards) {
                robot.roller.start(-0.75)
                robot.intake.start(0.30)
            } else {
                robot.roller.start(0.75)
                robot.intake.start(-0.30)
            }
        } else {
            robot.roller.start(0.0)
            robot.intake.start(0.0)
        }


        if (gamepad2.right_trigger > lastTriggerRight) {

            robot.shooter.shoot()

        }


        if (gamepad1.dpad_up) {

            driveSpeed = DriveSpeeds.Normal

        } else if (gamepad1.dpad_down) {

            driveSpeed = DriveSpeeds.Slow

        }


        if (gamepad2.left_trigger > 0.3 && gamepad2.a) {

            robot.shooter.simpleShootAtTarget(Pose2d(0.0, 0.0, 0.0), shootingGoal(70.0, 0.0, 32.0))

        } else if (gamepad2.left_trigger > 0.3) {

            robot.shooter.simpleShootAtTarget(Pose2d(0.0, 0.0, 0.0), shootingGoal(70.0, 0.0, 34.0))
//            robot.shooter.simpleShootAtTarget(robot.localizer.poseEstimate, Positions.highGoalRed)

        } else {

            robot.shooter.stopWheel()

        }

        if (gamepad2.dpad_up) {
            robot.arm.grabTele()
        } else if (gamepad2.dpad_down) {
            robot.arm.dropTele()
        }

        if (gamepad1.x && !previousGamepad1X) {
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
        previousGamepad1X = gamepad1.x




        try {
            //output values for robot movement
            var wheelSpeeds = DriveTrain.Vector(hori, vert, turn).speeds()
            wheelSpeeds = if (driveSpeed == DriveSpeeds.Normal) {
                DriveTrain.multiplySquare(speeds = wheelSpeeds, scalar = normalSpeed)
            } else {
                DriveTrain.multiplySquare(speeds = wheelSpeeds, scalar = slowSpeed)
            }
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