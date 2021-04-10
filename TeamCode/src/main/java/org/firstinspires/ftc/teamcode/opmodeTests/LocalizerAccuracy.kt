package org.firstinspires.ftc.teamcode.opmodeTests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Controllers.MecanumDriveTrain
import org.firstinspires.ftc.teamcode.robotConfigs.SparkyV2Robot


/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */


@Config
@TeleOp(group = "Opmodes for testing", name = "Localizer Accuracy")
class LocalizerAccuracy : LinearOpMode() {
    val ROBOT_RADIUS = 9.0

    @Throws(InterruptedException::class)
    override fun runOpMode() {
//        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val robot = SparkyV2Robot(hardwareMap, telemetry) { true }
        val drive = robot.driveTrain


        waitForStart()
        while (!isStopRequested) {
            robot.localizer.update()

            val vel = Pose2d(
                    (-gamepad1.left_stick_y).toDouble(),
                    (gamepad1.left_stick_x).toDouble(),
                    (gamepad1.right_stick_x).toDouble()
            )

            vel.times(0.5)

            drive.start(MecanumDriveTrain.Vector(vel.y, vel.x, vel.heading).speeds())

            val packet = TelemetryPacket()
            FtcDashboard.getInstance().telemetry.addData("localizer", robot.localizer.poseEstimate)

            val fieldOverlay: Canvas = packet.fieldOverlay()
            fieldOverlay.setStroke("#3F51B5")
            drawRobot(fieldOverlay, robot.localizer.poseEstimate)
            FtcDashboard.getInstance().sendTelemetryPacket(packet)
        }
    }

    fun drawRobot(canvas: Canvas, pose: Pose2d) {
        canvas.strokeCircle(pose.x, pose.y, ROBOT_RADIUS)
        val (x, y) = pose.headingVec().times(ROBOT_RADIUS)
        val x1 = pose.x + x / 2
        val y1 = pose.y + y / 2
        val x2 = pose.x + x
        val y2 = pose.y + y
        canvas.strokeLine(x1, y1, x2, y2)
    }
}