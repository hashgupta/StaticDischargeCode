package org.firstinspires.ftc.teamcode.tests

import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class DrivetrainTest {
    @Test
    fun testDrivetrain() {
        println(DriveTrain.Direction(0.0, 0.0, 1.5).speeds().rb)
//        println(DriveTrain.Direction(0.0, -1.0, 0.0).speeds().lf)
//        println(DriveTrain.Direction(0.0, -1.0, 0.0).speeds().rb)
//        println(DriveTrain.Direction(0.0, -1.0, 0.0).speeds().rf)
    }
}