package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.controllers.ShootingGoal
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
        val target = ShootingGoal(75.0, 0.0, 35.0)
        val shooterHeight = 8.0
        val shooterAngle = Math.toRadians(25.0)
        val slip = 1.000

        val g = 386.088583

        val position = pose.vec()
        val targetVector = Vector2d(target.x, target.y)
        val shotDistance = position distTo targetVector
        val netHeight = target.height - shooterHeight
        val requiredVelocity = sqrt(g / 2) * shotDistance / (cos(shooterAngle) * sqrt(shotDistance * tan(shooterAngle) - netHeight))
        // adjust slip for air resistance
        // since the longer the shot distance, the more work air resistance applies against the projectile
        // thus, we need proportionally more slip to compensate
        // the small number needs to be tuned
        val adjustedSlip = if (shotDistance > 73.0) {
            slip + ((shotDistance - 72) * 0.00375)
        } else {
            slip
        }

        val velocity = 2 * requiredVelocity * adjustedSlip
        val r = 2.0
//        val tpr = 28
        val gr = 1.0
        var angularVelocity = velocity / (r * gr) //radians/s
        angularVelocity /= (2 * PI) // rev/s
//        angularVelocity = angularVelocity * tpr //ticks/s
        angularVelocity *= 60
        println("rpm speed: $angularVelocity")
    }
}