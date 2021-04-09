package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class LinearLiftTest {
    @Test
    fun testLiftController() {
        val position = 10.0
        val velocity = 40.0
        val profile = MotionProfileGenerator.generateSimpleMotionProfile(MotionState(position, velocity), MotionState(50.0, 0.0), 50.0, 30.0)
        print(profile.duration())
    }
}