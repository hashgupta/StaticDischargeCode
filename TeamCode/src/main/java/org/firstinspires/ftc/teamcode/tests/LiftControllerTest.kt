package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import org.firstinspires.ftc.teamcode.Controllers.LiftController
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.purePursuit.*
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class LiftControllerTest {
    @Test
    fun testLiftController() {
        val position = 10.0
        val velocity = 40.0
        val profile = MotionProfileGenerator.generateSimpleMotionProfile(MotionState(position, velocity), MotionState(50.0,0.0), 50.0, 30.0)
        print(profile[1.3])
    }
}