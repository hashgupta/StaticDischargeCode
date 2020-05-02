package org.firstinspires.ftc.teamcode.tests


import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import org.firstinspires.ftc.teamcode.purePursuit.PurePursuitDrive
import org.firstinspires.ftc.teamcode.purePursuit.constants
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import kotlin.math.PI
import kotlin.math.abs

private val INITIAL_POSE = Pose2d(13.5, 12.0, 0.0)

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class PurePursuitTest {
    @Test
    fun testPurePursuitLibrary() {
        val localizer = MecanumDrive.MecanumLocalizer(object : MecanumDrive(0.0, 0.0, 0.0, 1.0) {
            override fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double) {
                throw UnsupportedOperationException()
            }

            override fun getWheelPositions() = listOf(0.0, 0.0, 0.0, 0.0)

            override val rawExternalHeading = 0.01
        })

        localizer.poseEstimate = INITIAL_POSE

        val drive = PurePursuitDrive(localizer)

        drive.addPoint(15.0,15.0,0.0)

        val goal = drive.findGoalPointAbsolute(localizer.poseEstimate, localizer.poseEstimate, drive.waypoints[0])
        print(drive.getWheelVelocityFromTarget(goal, localizer.poseEstimate, localizer.poseEstimate.vec() distTo drive.waypoints[0].vec(), 0.0, 1.0))

//        assert(localizer.poseEstimate epsilonEqualsHeading INITIAL_POSE)
    }
}