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

        val robot = SparkyV2Robot(hardwareMap, telemetry) { true }


        waitForStart()

        var rollingCount = 0
        val timer = ElapsedTime()
        while (!isStopRequested) {
            robot.localizer.update()
            rollingCount++

            if (timer.seconds() > 10) {
                timer.reset()
                rollingCount = 0

            } else if (timer.seconds() > 5) {
                telemetry.addData("average update time (ms)", timer.milliseconds() / rollingCount)
                telemetry.update()
            }

        }
    }
}