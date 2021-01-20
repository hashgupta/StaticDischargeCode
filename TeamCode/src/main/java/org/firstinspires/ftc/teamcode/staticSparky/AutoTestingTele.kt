package org.firstinspires.ftc.teamcode.staticSparky

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.firstinspires.ftc.teamcode.Positions
import org.firstinspires.ftc.teamcode.robotConfigs.SparkyRobot
import org.firstinspires.ftc.teamcode.robotConfigs.SparkyV2Robot

@TeleOp(name = "AutoTestingTele", group = "StaticDischarge")
class AutoTestingTele : SparkOpModeBase() {

    lateinit var robot: SparkyRobot


    private var previousGamepad1Guide: Boolean = false
    private var previousGamepad1RT: Double = 0.0
    private var driveSpeed = 1.0
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

    private fun startRobot() {
        timer.reset()
    }

    private fun initRobot() {
        //initialize and set robot behavior
        robot = SparkyRobot(hardwareMap, telemetry) { true }
        robot.driveTrain.setZeroBehavior(DcMotor.ZeroPowerBehavior.FLOAT)

    }

    private fun loopRobot() {
        robot.localizer.update()

        // *******************
        // GAMEPAD 1 controls
        // *******************

        // get gamepad input
        // moving the joystick up is actually negative, not positive, so use negative to flip it
        val vert = -gamepad1.left_stick_y.toDouble()
        val hori = gamepad1.left_stick_x.toDouble()
        val turn = gamepad1.right_stick_x.toDouble()


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




        if (gamepad1.left_trigger > 0.3) {
            robot.shooter.simpleShootAtTarget(robot.localizer.poseEstimate, Positions.highGoalRed)
        } else {
            robot.shooter.stopWheel()
        }

        if (gamepad1.right_trigger > 0.5 && previousGamepad1RT < 0.5) {
            robot.shooter.shoot()
        }

        // ******************
        // run robot with movements
        // update control values
        // ******************


        previousGamepad1Guide = gamepad1.guide

        previousGamepad1RT = gamepad1.right_trigger.toDouble()



        try {
//            //output values for robot movement
            robot.driveTrain.start(DriveTrain.Vector(
                    hori * driveSpeed,
                    vert * driveSpeed,
                    turn * driveSpeed)
                    .speeds())
            telemetry.addData("position", robot.localizer.poseEstimate)
            telemetry.update()


        } catch (e: Exception) {
            telemetry.addData("Error", e.message)
            telemetry.addData("info", e.stackTrace[0].toString())
            telemetry.update()
        }

    }


    private fun stopRobot() {
        robot.driveTrain.start(DriveTrain.Vector(0.0, 0.0, 0.0).speeds())
    }
}