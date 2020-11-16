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
        val vert = 23.0
        println(DriveTrain.Direction(hori, -vert, 0.0).speeds())


//        println(DriveTrain.Direction(0.0, -1.0, 0.0).speeds().lf)
//        println(DriveTrain.Direction(0.0, -1.0, 0.0).speeds().rb)
//        println(DriveTrain.Direction(0.0, -1.0, 0.0).speeds().rf)
    }
}