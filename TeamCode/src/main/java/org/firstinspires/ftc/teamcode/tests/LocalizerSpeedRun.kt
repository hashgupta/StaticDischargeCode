package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.firstinspires.ftc.teamcode.robotConfigs.SparkyV2Robot
import org.firstinspires.ftc.teamcode.robotConfigs.TestRobot
import kotlin.math.abs

@Config
@TeleOp(group = "tests")
class LocalizerSpeedRun : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
//        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val robot = SparkyV2Robot(hardwareMap, telemetry) { true }
        val drive = robot.driveTrain

        // move forward at moderate speed
        val baseVel = Pose2d(
                (0.0 ),
                (0.0),
                (0.0)
        )

        val vel = if (abs(baseVel.x) + abs(baseVel.y) + abs(baseVel.heading) > 1) {
            // re-normalize the powers according to the weights
            val denom = VX_WEIGHT * abs(baseVel.x) + VY_WEIGHT * abs(baseVel.y) + OMEGA_WEIGHT * abs(baseVel.heading)
            Pose2d(
                    VX_WEIGHT * baseVel.x,
                    VY_WEIGHT * baseVel.y,
                    OMEGA_WEIGHT * baseVel.heading
            ).div(denom)
        } else {
            baseVel
        }
        val wheelVels = MecanumKinematics.robotToWheelVelocities(vel, 18.0, 18.0, 1.0)
        drive.start(DriveTrain.Square(wheelVels[3], wheelVels[2], -wheelVels[0], -wheelVels[1]))

        waitForStart()

        var rollingCount = 0
        val timer = ElapsedTime()
        while (!isStopRequested) {
            robot.localizer.update()
            rollingCount++
            telemetry.addData("timer: ", timer)
            if (timer.seconds() > 5) {
                telemetry.addData("average update time (ms)", timer.milliseconds() / rollingCount)
                timer.reset()
                rollingCount = 0
                telemetry.update()
                sleep(1000)
            }
            if (gamepad1.a) {
                break
            }
            telemetry.update()
        }
    }
    companion object {
        var VX_WEIGHT = 1.0
        var VY_WEIGHT = 1.0
        var OMEGA_WEIGHT = 1.0
    }
}