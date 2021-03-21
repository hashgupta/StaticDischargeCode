package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.Controllers.shootingGoal
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sqrt
import kotlin.math.tan

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class ShooterTest {
    @Test
    fun testShooterMath() {
        val pose = Pose2d(0.0, 0.0, 0.0)
        val target = shootingGoal(86.0, 0.0, 35.0)
        val shooterHeight = 8.0
        val shooterAngle = Math.toRadians(45.0)
        val slip = 1.52

        val g = 386.088583

        val position = pose.vec()
        val targetVector = Vector2d(target.x, target.y)
        val shotDistance = position distTo targetVector
        val net_height = 36 - -shooterHeight
        val requiredVelocity = Math.sqrt(g / 2) * shotDistance / (cos(shooterAngle) * sqrt(shotDistance * tan(shooterAngle) - net_height))
        println((requiredVelocity).toString())
        println((2 * requiredVelocity * slip / (2 * 2 * PI)) * 60)
        println((2 * requiredVelocity * slip / (2 * 2 * PI)) * 28)

        val velocity = 2 * requiredVelocity * slip
        val r = 2.0
        val tpr = 28
        val gr = 1.0
        var angularVelocity = velocity / (r * gr) //radians/s
        angularVelocity = angularVelocity / (2 * PI) // rev/s
        angularVelocity = angularVelocity * tpr //ticks/s
        println("ticks/s speed: " + angularVelocity)
    }
}