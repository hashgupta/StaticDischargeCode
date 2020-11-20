package org.firstinspires.ftc.teamcode.tests

import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import kotlin.math.PI

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class DrivetrainTest {
    @Test
    fun testDrivetrain() {
        val hori = 0.0
        val vert = 1.0
        println(DriveTrain.Vector(hori, vert, 0.0).speeds())
    }
}