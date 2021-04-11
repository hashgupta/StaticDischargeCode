package org.firstinspires.ftc.teamcode.matchopmodes


import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.controllers.MecanumDriveTrain
import org.firstinspires.ftc.teamcode.controllers.ShootingGoal
import org.firstinspires.ftc.teamcode.robotconfigs.SparkyV2Robot
import kotlin.math.abs

@TeleOp(name = "GigaWatt TeleOp", group = "StaticDischarge")
class GigaWattTeleOp : GenericOpModeBase() {
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

    private var previousGamepad1Y = false


    private var IntakeBackwards = false

    private val normalSpeed = 0.95
    private val slowSpeed = 0.3

    var intakeRollerSpeed = 0.80

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
        var vert: Double = -gamepad1.left_stick_y.toDouble()
        var hori: Double = gamepad1.left_stick_x.toDouble() * 1.1
        val turn: Double = gamepad1.right_stick_x.toDouble() * 0.9
        val wobble: Double = -gamepad2.right_stick_y.toDouble()

        if (abs(vert) < 0.1) {
            vert = 0.0
        }

        if (abs(hori) < 0.1) {
            hori = 0.0
        }


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


        if ((gamepad2.right_trigger > lastTriggerRight || gamepad2.right_trigger > 0.99) && (gamepad2.right_trigger > 0.10)) {
            robot.driveTrain.start(MecanumDriveTrain.Square(0.0, 0.0, 0.0, 0.0))
            robot.shooter.shoot()
            return
        }


        if (gamepad1.dpad_up) {

            driveSpeed = DriveSpeeds.Normal

        } else if (gamepad1.dpad_down) {

            driveSpeed = DriveSpeeds.Slow

        }


        if (gamepad2.left_trigger > 0.3 && gamepad2.a) {


            robot.shooter.aimShooter(Pose2d(0.0, 0.0, 0.0), ShootingGoal(70.0, 0.0, 32.25))

        } else if (gamepad2.left_trigger > 0.3) {

            robot.shooter.aimShooter(Pose2d(0.0, 0.0, 0.0), ShootingGoal(77.0, 0.0, 35.0))


        } else {

            robot.shooter.stopWheel()

        }

        if (gamepad2.dpad_up) {
            robot.arm.grabTele()
        } else if (gamepad2.dpad_down) {
            robot.arm.dropTele()
        }

        if (gamepad1.y && !previousGamepad1Y) {
            robot.pursuiter.startAt(robot.localizer.poseEstimate)
            robot.pursuiter.action { robot.pursuiter.runSpeed *= 0.8 }
            robot.pursuiter.relative(7.5, 0.0, 0.0)
            robot.pursuiter.action { robot.pursuiter.runSpeed *= 1.25 }
            robot.pursuiter.follow(robot.driveTrain, telemetry = telemetry)
        }

        if (gamepad1.right_trigger.toDouble() > 0.8) {
            robot.flicker.start(0.65)
        }



        lastTriggerRight = gamepad2.right_trigger.toDouble()

        previousGamepad1Y = gamepad1.y




        try {

            //output values for robot movement
            robot.arm.run(wobble)

            var wheelSpeeds = MecanumDriveTrain.Vector(hori, vert, turn).speeds()

            wheelSpeeds = if (driveSpeed == DriveSpeeds.Normal) {
                MecanumDriveTrain.multiplySquare(speeds = wheelSpeeds, scalar = normalSpeed)
            } else {
                MecanumDriveTrain.multiplySquare(speeds = wheelSpeeds, scalar = slowSpeed)
            }

            robot.driveTrain.start(wheelSpeeds)

            telemetry.addData("Pose Estimate", robot.localizer.poseEstimate)
            telemetry.update()


        } catch (e: Exception) {
            telemetry.addData("Error", e.message)
            telemetry.addData("Info", e.stackTrace[0].toString())
            telemetry.update()

        }

    }

    fun stopRobot() {
        robot.driveTrain.start(MecanumDriveTrain.Vector(0.0, 0.0, 0.0).speeds())
    }
}