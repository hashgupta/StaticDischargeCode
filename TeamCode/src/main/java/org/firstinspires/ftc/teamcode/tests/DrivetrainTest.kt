package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.util.Angle
import org.firstinspires.ftc.teamcode.Positions
import org.firstinspires.ftc.teamcode.localizers.MockedLocalizer
import org.firstinspires.ftc.teamcode.purePursuit.FastPurePursuit
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import kotlin.math.PI

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class DrivetrainTest {
    @Test
    fun testDrivetrain() {
        val localizer = MockedLocalizer()
        localizer.poseEstimate = Pose2d(0.0,0.0,Math.toRadians(153.0))
        val pursuiter = FastPurePursuit(localizer)

//        val pose = Pose2d(0.0, 0.0, 0.0)
//        val target = shootingGoal(86.0, 0.0, 35.0)
//        val shooterHeight = 8.0
//        val shooterAngle = Math.toRadians(45.0)
//        val slip = 1.52
//
//        val position = pose.vec()
//        val targetVector = Vector2d(target.x, target.y)
//        val shotDistance = position distTo targetVector
//        val net_height = 36 - 8.0
//        val requiredVelocity = Math.sqrt(g /2) * shotDistance/( cos(shooterAngle) * sqrt( shotDistance * tan(shooterAngle) - net_height))
//        println((requiredVelocity).toString())
//        println((2*requiredVelocity*slip / (2*2*PI)) * 60)
//        println((2*requiredVelocity*slip / (2*2*PI)) * 28)
//
//        val velocity = 2*requiredVelocity * slip
//        val r = 2.0
//        val tpr = 28
//        val gr = 1.0
//        var angularVelocity = velocity/(r*gr) //radians/s
//        angularVelocity = angularVelocity / (2*PI) // rev/s
//        angularVelocity = angularVelocity * tpr //ticks/s
//        println("ticks/s speed: " + angularVelocity)




//        val pose = Pose2d(0.4, 0.3, Math.toRadians(180.55))
//        val target = Pose2d(0.0,0.0,Math.toRadians(180.0))
//        val poseError = Kinematics.calculatePoseError(target, pose)
//        val translationalTol = 1.0
//
//        val angularTol = Math.toRadians(0.5)
//
//        if (abs(poseError.x) < translationalTol && abs(poseError.y) < translationalTol &&
//                abs(poseError.heading) < angularTol) {
//            // go to next waypoint
//            println("next waypoint")
//        }
//
//        pursuiter.setStartPoint(pose)
//        pursuiter.addPoint(target)
//
//
//
//        println("velocity"+pursuiter.getVelocityFromTarget(target, pose))
//
//        println("pose error: "+ Kinematics.calculatePoseError(target, pose))
//        println(pursuiter.followStepTestWriteup())


        pursuiter.setStartPoint(localizer.poseEstimate)
        val position = localizer.poseEstimate.vec()

        val target = Positions.highGoalRed
        val targetVector = Vector2d(target.x, target.y)
        val shootingHeadingVector = targetVector.minus(position)

        val targetAngle = shootingHeadingVector.angle() + PI
        println(Angle.norm(targetAngle))
        pursuiter.addTurnAbsolute(
                targetAngle)

        pursuiter.followStepTestWriteup()


    }
}