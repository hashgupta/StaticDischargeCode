package org.firstinspires.ftc.teamcode.tests


import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.purePursuit.*
import org.junit.jupiter.api.Disabled
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance

private val INITIAL_POSE = Pose2d(20.0, 21.0, 0.0)

@Disabled
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class PurePursuitTest {
    @Test
    fun testPurePursuitLibrary() {
        val localizer = MecanumDrive.MecanumLocalizer(object : MecanumDrive(0.0, 0.0, 0.0, 1.0) {
            override fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double) {
                throw UnsupportedOperationException()
            }

            override fun getWheelPositions() = listOf(0.0, 0.0, 0.0, 0.0)

//            override fun getWheelVelocities(): List<Double>? {
//                return listOf(0.0, 0.0, 0.0, 0.0)
//            }

            override val rawExternalHeading = 0.01
        })

        localizer.poseEstimate = INITIAL_POSE

        val drive = SimplePathPurePursuit(localizer, INITIAL_POSE)

        drive.addSpline(Pose2d(30.0, 30.0, Math.toRadians(90.0)),Math.toRadians(0.0),Math.toRadians(180.0))

//        drive.waypoints.add(0, localizer.poseEstimate)


//        val goal = limit(drive.waypoints[0].findClosestT(localizer.poseEstimate) + 5/drive.waypoints[0].length, 0.0, 1.0)
    }
}