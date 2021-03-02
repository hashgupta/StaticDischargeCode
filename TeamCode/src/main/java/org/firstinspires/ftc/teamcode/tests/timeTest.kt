package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import org.firstinspires.ftc.teamcode.Constants
import org.junit.jupiter.api.Disabled
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance

private val INITIAL_POSE = Pose2d(13.0, 13.0, 0.0)

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
@Disabled
class timeTest {
    @Test
    fun testPurePursuitLibrary() {

        val wheelPositions = arrayListOf<Double>(0.0, 0.0, 5.0, 5.0)
        val lastWheelPositions = arrayListOf(0, 0, 0, 0)
        val wheelDeltas = wheelPositions
                .zip(lastWheelPositions)
                .map { it.first - it.second }
        val robotPoseDelta = MecanumKinematics.wheelToRobotVelocities(
                wheelDeltas, Constants.trackwidth, Constants.wheelBase, 1.0
        )
        println(robotPoseDelta)

    }
}