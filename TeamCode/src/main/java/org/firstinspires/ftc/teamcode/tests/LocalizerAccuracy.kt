package org.firstinspires.ftc.teamcode.tests
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.firstinspires.ftc.teamcode.robotConfigs.TestRobot
import org.firstinspires.ftc.teamcode.localizers.MecanumLocalizerRev
import org.firstinspires.ftc.teamcode.robotConfigs.SparkyV2Robot
import kotlin.math.abs


/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Disabled
@Config
@TeleOp(name = "Localizer Accuracy",group = "tests")
class LocalizerAccuracy : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
//        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val robot = SparkyV2Robot(hardwareMap, telemetry) { true }
        val drive = robot.driveTrain
        

        waitForStart()
        while (!isStopRequested) {
            val baseVel = Pose2d(
                    (-gamepad1.left_stick_y).toDouble(),
                    (gamepad1.left_stick_x).toDouble(),
                    (gamepad1.right_stick_x).toDouble()
            )

//            drive.start(DriveTrain.Vector(vel.y, vel.x, vel.heading).speeds())
//            telemetry.addData("velocities", vel)
//            telemetry.addData("wheel deltas", (robot.localizer as MecanumLocalizerRev).getWheelPositions())
            robot.localizer.update()
            val (x, y, heading) = robot.localizer.poseEstimate
            telemetry.addData("movement velocity", robot.localizer.poseVelocity)
            telemetry.addData("x", x)
            telemetry.addData("y", y)
            telemetry.addData("heading", heading)
            telemetry.update()
        }
    }

    companion object {
        var VX_WEIGHT = 0.5
        var VY_WEIGHT = 0.5
        var OMEGA_WEIGHT = 0.5
    }
}