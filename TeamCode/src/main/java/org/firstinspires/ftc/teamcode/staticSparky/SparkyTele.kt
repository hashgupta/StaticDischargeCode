package org.firstinspires.ftc.teamcode.staticSparky

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ReadWriteFile
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.firstinspires.ftc.teamcode.Controllers.shootingGoal
import java.io.File
import kotlin.math.abs

@TeleOp(name = "SparkyTele", group = "StaticDischarge")
class SparkyTele : LinearOpMode() {
    // robot
    private lateinit var robot: SparkyRobot
    private var reverse = false

    // speeds
    private var driveSpeed = 1.0

    private var shooterIsSpinning: Boolean = false
    private var previousGamepad1X: Boolean = false
    private var previousGamepad1LBumper: Boolean = false
    private var intakeForwards: Boolean = false
    private var intakeOFF = true
    private var previousGamepad1RBumper: Boolean = false

    override fun runOpMode() {
        initRobot()
        waitForStart()
        while (opModeIsActive()) {
            loopRobot()
        }
        stopRobot()
    }

    fun initRobot() {
        //initialize and set robot behavior
        robot = SparkyRobot(hardwareMap, telemetry) {true}
        robot.driveTrain.setZeroBehavior(DcMotor.ZeroPowerBehavior.FLOAT)

        try {
            val filename = "position.json"
            val file: File = AppUtil.getInstance().getSettingsFile(filename)

            val positionString = ReadWriteFile.readFile(file)
            val positionValues = positionString.split(" ")
            val robot_pose = Pose2d(positionValues[0].toDouble(),positionValues[1].toDouble(), positionValues[2].toDouble())
            robot.pursuiter.setStartPoint(robot_pose)
            robot.pose = robot_pose
        } catch (e:Exception) {
            telemetry.addLine(e.toString())
            telemetry.update()
        }
    }

    fun loopRobot() {
        robot.localizer.update()

        // get gamepad input
        var vert = gamepad1.left_stick_y.toDouble()
        var hori = gamepad1.left_stick_x.toDouble()
        val turn = gamepad1.right_stick_x.toDouble()
        var wobble = gamepad1.right_stick_y.toDouble()

        // process input

        if (gamepad1.left_bumper && !previousGamepad1LBumper) {
            intakeForwards = !intakeForwards

        }
        if (gamepad1.right_bumper && !previousGamepad1RBumper) {
            intakeOFF = !intakeOFF
            intakeForwards = true
        }


        if (intakeOFF) {
            robot.intakeBottom.start(0.0)
            robot.intakeTop.start(0.0)
        }
        else if (intakeForwards) {
            robot.intakeBottom.start(0.75)
            robot.intakeTop.start(1.0)
        } else {
            robot.intakeBottom.start(-0.75)
            robot.intakeTop.start(-1.0)
        }

        if (gamepad1.dpad_up) {
            robot.arm.grabTele()
        } else if (gamepad1.dpad_down) {
            robot.arm.dropTele()
        } else {
            robot.arm.stopGrabber()
        }




        if (gamepad1.x && !previousGamepad1X) {
            shooterIsSpinning = !shooterIsSpinning

        }

        if (shooterIsSpinning) {
            robot.shooter.simpleShootAtTarget(Pose2d(0.0, 0.0, 0.0), shootingGoal(70.0, 0.0, 35.0))
//            robot.shooter.simpleShootAtTarget(robot.localizer.poseEstimate, Positions.highGoalRed)
        } else {
            robot.shooter.stopWheel()
        }

        if (gamepad1.right_trigger > 0.5) {
            vert = 0.0
            hori = 0.0
            robot.shooter.shoot()
        } else {
            robot.shooter.stopShoot()
        }

        previousGamepad1X = gamepad1.x
        previousGamepad1RBumper = gamepad1.right_bumper
        previousGamepad1LBumper= gamepad1.left_bumper



        if (abs(vert) < 0.1) {
            vert = 0.0
        }
        if (abs(hori) < 0.1) {
            hori = 0.0
        }
        if (abs(wobble) < 0.3) {
            wobble = 0.0
        }

        try {
//            //output values for robot movement
            robot.driveTrain.start(DriveTrain.Vector(
                    hori * driveSpeed * (if (reverse) -1 else 1).toDouble(),
                    vert * driveSpeed * (if (reverse) -1 else 1).toDouble(),
                    turn * driveSpeed)
                    .speeds())
            robot.arm.run(wobble)
            telemetry.addData("gyro", robot.gyro.measure())
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