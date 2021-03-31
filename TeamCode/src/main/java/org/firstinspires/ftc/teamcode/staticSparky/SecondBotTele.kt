package org.firstinspires.ftc.teamcode.staticSparky


import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.firstinspires.ftc.teamcode.Controllers.shootingGoal
import org.firstinspires.ftc.teamcode.Positions
import org.firstinspires.ftc.teamcode.robotConfigs.SparkyV2Robot

@TeleOp(name = "Second Robot Tele", group = "StaticDischarge")
class SecondBotTele : GenericOpModeBase() {
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
//    private var previousGamepad1X = false
    private var previousGamepad1Y = false
//    private var previousGamepad2Y = false
    private var IntakeBackwards = false

    private val normalSpeed = 0.95
    private val slowSpeed = 0.3
//    private var aimBotOn = false

    @JvmField
    var intakeRollerSpeed = 0.90
    @JvmField
    var intakeMainSpeed = -0.90

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
        val vert: Double = -gamepad1.left_stick_y.toDouble()
        val hori: Double = gamepad1.left_stick_x.toDouble() * 1.1
        val turn: Double = gamepad1.right_stick_x.toDouble() * 0.9
        val wobble: Double = -gamepad2.right_stick_y.toDouble()

        // process input
        if (gamepad1.a) {

            IntakeOn = true

        } else if (gamepad1.b) {

            IntakeOn = false
        }


        if (gamepad1.right_bumper) {
            IntakeBackwards = true
        } else if (gamepad1.left_bumper) {
            IntakeBackwards = false
        }


        if (IntakeOn) {
            if (IntakeBackwards) {
                robot.roller.start(-intakeRollerSpeed)
                robot.intake.start(-intakeMainSpeed)
            } else {
                robot.roller.start(intakeRollerSpeed)
                robot.intake.start(intakeMainSpeed)
            }
        } else {
            robot.roller.start(0.0)
            robot.intake.start(0.0)
        }




//        if (gamepad2.left_bumper) {
//            aimBotOn = false
//        } else if (gamepad2.right_bumper) {
//            aimBotOn = true
//        }
        

        if (gamepad2.right_trigger > lastTriggerRight || gamepad2.right_trigger > 0.99) {
            robot.driveTrain.start(DriveTrain.Square(0.0, 0.0, 0.0, 0.0))
            robot.shooter.shoot()
            return
        }


        if (gamepad1.dpad_up) {

            driveSpeed = DriveSpeeds.Normal

        } else if (gamepad1.dpad_down) {

            driveSpeed = DriveSpeeds.Slow

        }


        if (gamepad2.left_trigger > 0.3 && gamepad2.a) {


            robot.shooter.aimShooter(Pose2d(0.0, 0.0, 0.0), shootingGoal(70.0, 0.0, 32.5))

        } else if (gamepad2.left_trigger > 0.3) {

//            if (aimBotOn) robot.shooter.aimShooter(robot.localizer.poseEstimate, Positions.highGoalRed)
//            else robot.shooter.aimShooter(Pose2d(0.0, 0.0, 0.0), shootingGoal(75.0, 0.0, 35.0))
            robot.shooter.aimShooter(Pose2d(0.0, 0.0, 0.0), shootingGoal(75.0, 0.0, 35.0))


        } else {

            robot.shooter.stopWheel()

        }

        if (gamepad2.dpad_up) {
            robot.arm.grabTele()
        } else if (gamepad2.dpad_down) {
            robot.arm.dropTele()
        }

//        if (gamepad1.x && !previousGamepad1X) {
//            robot.pursuiter.startAt(robot.localizer.poseEstimate)
//            robot.pursuiter
//                    .turnTo(robot.shooter.turningTarget(robot.localizer.poseEstimate.vec(), Positions.highGoalRed))
//                    .follow(robot.driveTrain, telemetry = telemetry)
//        }
//
        if (gamepad1.y && !previousGamepad1Y) {
            robot.pursuiter.startAt(robot.localizer.poseEstimate)
            robot.pursuiter.action { robot.pursuiter.runSpeed *= 0.5 }
            robot.pursuiter.relative(7.5, 0.0, 0.0)
            robot.pursuiter.action { robot.pursuiter.runSpeed *= 2 }
            robot.pursuiter.follow(robot.driveTrain, telemetry = telemetry)
        }
//
        if (gamepad1.right_trigger.toDouble() == 1.0) {
            robot.flicker.start(0.65)
        }
//
//
//        //auto aim and shoot three shots, no human involvement
//        if (gamepad2.y && !previousGamepad2Y) {
//            robot.pursuiter.startAt(robot.localizer.poseEstimate)
//
//            robot.pursuiter
//                    .turnTo(robot.shooter.turningTarget(robot.localizer.poseEstimate.vec(), Positions.highGoalRed))
//                    .follow(robot.driveTrain, telemetry = telemetry)
//
//            robot.shooter.aimShooter(robot.localizer.poseEstimate, Positions.highGoalRed)
//            sleep(1000)
//            robot.shooter.shoot()
//            sleep(250)
//            robot.shooter.shoot()
//            sleep(250)
//            robot.shooter.shoot()
//
//        }


        lastTriggerRight = gamepad2.right_trigger.toDouble()
//        previousGamepad1X = gamepad1.x
//        previousGamepad2Y = gamepad2.y
        previousGamepad1Y = gamepad1.y




        try {

            //output values for robot movement
            robot.arm.run(wobble)

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