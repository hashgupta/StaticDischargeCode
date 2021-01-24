package org.firstinspires.ftc.teamcode.staticSparky

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.ReadWriteFile
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.firstinspires.ftc.teamcode.Controllers.shootingGoal
import org.firstinspires.ftc.teamcode.Positions
import org.firstinspires.ftc.teamcode.robotConfigs.SparkyRobot
import java.io.File
import kotlin.math.abs

const val TILE_LENGTH = 24.0

@TeleOp(name = "SparkyTele", group = "StaticDischarge")
class SparkyTele : SparkOpModeBase() {
    // robot
    lateinit var robot: SparkyRobot
    private var reverse = false

    // speeds
    private var driveSpeed = 1.0

    private var previousGamepad1Guide: Boolean = false
    private var previousGamepad1LBumper: Boolean = false
    private var intakeForwards: Boolean = false
    private var intakeOFF = true
    private var previousGamepad1RBumper: Boolean = false
    private var previousGamepad2RT: Double = 0.0
    private val timer:ElapsedTime = ElapsedTime()

    override fun runOpMode() {
        initRobot()
        waitForStart()
        startRobot()
        while (opModeIsActive()) {
            loopRobot()
        }
        stopRobot()
    }

    fun startRobot() {
        timer.reset()
    }

    fun initRobot() {
        //initialize and set robot behavior
        robot = SparkyRobot(hardwareMap, telemetry) { true }
        robot.driveTrain.setZeroBehavior(DcMotor.ZeroPowerBehavior.FLOAT)



//        try {
//            val filename = "position.json"
//            val file: File = AppUtil.getInstance().getSettingsFile(filename)
//
//            val positionString = ReadWriteFile.readFile(file)
//            val positionValues = positionString.split(" ")
//            val robot_pose = Pose2d(positionValues[0].toDouble(),positionValues[1].toDouble(), positionValues[2].toDouble())
//            robot.localizer.poseEstimate = robot_pose
//        } catch (e:Exception) {
//            telemetry.addLine(e.toString())
//            telemetry.update()
//        }
        robot.loadPose()
    }

    fun loopRobot() {
        robot.localizer.update()

        // *******************
        // GAMEPAD 1 controls
        // *******************

        // get gamepad input
        var vert = gamepad1.left_stick_y.toDouble()
        var hori = gamepad1.left_stick_x.toDouble()
        val turn = gamepad1.right_stick_x.toDouble()


        if (abs(vert) < 0.1) {
            vert = 0.0
        }
        if (abs(hori) < 0.1) {
            hori = 0.0
        }

        if (gamepad1.b) {
            driveSpeed = 0.5
        }
        if (gamepad1.a) {
            driveSpeed = 1.0
        }


        //intake controls
        if (gamepad1.left_bumper && !previousGamepad1LBumper) {
            intakeForwards = !intakeForwards

        }
        if (gamepad1.right_bumper && !previousGamepad1RBumper) {
            intakeOFF = !intakeOFF
            intakeForwards = true
        }

        // set intake
        if (intakeOFF) {
            robot.intakeBottom.start(0.0)
            robot.intakeTop.start(0.0)
        }
        else if (intakeForwards) {
            robot.intakeBottom.start(1.0)
            robot.intakeTop.start(1.0)
        } else {
            robot.intakeBottom.start(-1.0)
            robot.intakeTop.start(-1.0)
        }

        // auto movement to shooting position
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

        // *******************
        // GAMEPAD 2 controls
        // *******************

        var wobble = gamepad2.right_stick_y.toDouble()

        if (abs(wobble) < 0.3) {
            wobble = 0.0
        }


        if (gamepad2.dpad_up) {
            robot.arm.grabTele()
        } else if (gamepad2.dpad_down) {
            robot.arm.dropTele()
        }


        if (gamepad2.left_trigger > 0.3) {
            robot.shooter.simpleShootAtTarget(Pose2d(0.0, 0.0, 0.0), shootingGoal(70.0, 0.0, 35.0))
//            robot.shooter.simpleShootAtTarget(robot.localizer.poseEstimate, Positions.highGoalRed)
        } else {
            robot.shooter.stopWheel()
        }

        if (gamepad2.right_trigger > 0.5 && previousGamepad2RT < 0.5) {
            robot.shooter.shoot()
        }

        // ******************
        // run robot with movements
        // update control values
        // ******************


        previousGamepad1Guide = gamepad1.guide
        previousGamepad1RBumper = gamepad1.right_bumper
        previousGamepad1LBumper= gamepad1.left_bumper
        previousGamepad2RT = gamepad2.right_trigger.toDouble()



        try {
//            //output values for robot movement
            robot.driveTrain.start(DriveTrain.Vector(
                    hori * driveSpeed * (if (reverse) -1 else 1).toDouble(),
                    vert * driveSpeed * (if (reverse) -1 else 1).toDouble(),
                    turn * driveSpeed)
                    .speeds())
            robot.arm.run(wobble)
            telemetry.addData("position", robot.localizer.poseEstimate)
            telemetry.update()
//            robot.lift.start(liftSpeed(lift))


        } catch (e: Exception) {
            telemetry.addData("Error", e.message)
            telemetry.addData("info", e.stackTrace[0].toString())
            telemetry.update()
        }

    }


    fun stopRobot() {
        robot.driveTrain.start(DriveTrain.Vector(0.0, 0.0, 0.0).speeds())
    }

    companion object {
        // field measurements

    }
}