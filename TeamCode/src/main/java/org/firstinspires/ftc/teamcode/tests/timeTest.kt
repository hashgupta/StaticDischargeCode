package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.purePursuit.PurePursuitDrive
import org.junit.jupiter.api.Disabled
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance

private val INITIAL_POSE = Pose2d(13.0, 13.0, 0.0)

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
@Disabled
class timeTest {
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

        for (i  in 0..6) {
            drive.addPoint(-46.0, 15.0, 0.0)
                    .addPoint(46.0, 15.0, 0.0)
        }

        print(drive.estimateTime(Pose2d(0.0,0.0,0.0)))
//        assert(localizer.poseEstimate epsilonEqualsHeading INITIAL_POSE)
    }
}