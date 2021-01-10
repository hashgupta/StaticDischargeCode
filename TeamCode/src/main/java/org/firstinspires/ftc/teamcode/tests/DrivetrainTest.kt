package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import org.firstinspires.ftc.teamcode.Controllers.g
import org.firstinspires.ftc.teamcode.localizers.MockedLocalizer
import org.firstinspires.ftc.teamcode.pipelines.Ring
import org.firstinspires.ftc.teamcode.purePursuit.FastPurePursuit
import org.firstinspires.ftc.teamcode.staticSparky.Positions
import org.firstinspires.ftc.teamcode.staticSparky.SparkOpModeBase
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import kotlin.math.*

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class DrivetrainTest {
    @Test
    fun testDrivetrain() {
        val pursuiter = FastPurePursuit(MockedLocalizer(), Pose2d(0.0,0.0,0.0))



//        val position = Vector2d(0.0, -SparkAutoBase.TILE_LENGTH *1.5)
//        val target = Positions.highGoalRed
//        val targetVector = Vector2d(target.x, target.y)
//        val shootingHeading = targetVector.minus(position).angle()
//        println(shootingHeading)
//        val pose = Pose2d(0.0, 0.0, 0.0)
//        val target = shootingGoal(80.0, 0.0, 35.0)
//        val shooterHeight = 8.0
//        val shooterAngle = Math.toRadians(45.0)
//        val slip = 1.0
//
//        val position = pose.vec()
//        val targetVector = Vector2d(target.x, target.y)
//        val shotDistance = 84
//        val net_height = 32 - 8.0
//        val requiredVelocity = Math.sqrt(g /2) * shotDistance/( cos(shooterAngle) * sqrt( shotDistance * tan(shooterAngle) - net_height))
//        println((requiredVelocity).toString())
//        println(2*requiredVelocity*slip / (2*2*PI) * 60)
//
//        val velocity = 2*requiredVelocity
//        val r = 2.0
//        val tpr = 28
//        val gr = 1.0
//        var angularVelocity = velocity/(r*gr) //radians/s
//        angularVelocity = angularVelocity / (2*PI) // rev/s
//        angularVelocity = angularVelocity * tpr //ticks/s
//        println("ticks/s speed: " + angularVelocity)




        val pose = Pose2d(0.4, 0.3, Math.toRadians(180.55))
        val target = Pose2d(0.0,0.0,Math.toRadians(180.0))
        val poseError = Kinematics.calculatePoseError(target, pose)
        val translationalTol = 1.0

        val angularTol = Math.toRadians(0.5)

        if (abs(poseError.x) < translationalTol && abs(poseError.y) < translationalTol &&
                abs(poseError.heading) < angularTol) {
            // go to next waypoint
            println("next waypoint")
        }

        pursuiter.setStartPoint(pose)
        pursuiter.addPoint(target)



        println("velocity"+pursuiter.getVelocityFromTarget(target, pose))

        println("pose error: "+ Kinematics.calculatePoseError(target, pose))
        println(pursuiter.followStepTestWriteup())


    }
}