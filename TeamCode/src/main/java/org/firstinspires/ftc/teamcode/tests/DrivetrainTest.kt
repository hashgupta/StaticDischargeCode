package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import kotlin.math.PI

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class DrivetrainTest {
    @Test
    fun testDrivetrain() {
        val goalPose = Pose2d(23.0, -23.0, 3.14159)
        val pose = Pose2d(0.0, 46.0, 0.0)
        val error = Kinematics.calculatePoseError(goalPose, pose)
        println(-error.y)
        println(error.x)
        println((error.heading+pose.heading) * 180 / PI)
    }
}