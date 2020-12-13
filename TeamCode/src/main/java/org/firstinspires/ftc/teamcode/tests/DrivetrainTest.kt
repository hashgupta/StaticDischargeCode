package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.qualcomm.hardware.motors.RevRoboticsUltraPlanetaryHdHexMotor
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.firstinspires.ftc.teamcode.Controllers.g
import org.firstinspires.ftc.teamcode.Controllers.shootingGoal
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.localizers.MockedLocalizer
import org.firstinspires.ftc.teamcode.pipelines.RingPipeline
import org.firstinspires.ftc.teamcode.purePursuit.FastPurePursuit
import org.firstinspires.ftc.teamcode.staticSparky.Positions
import org.firstinspires.ftc.teamcode.staticSparky.SparkAutoBase
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sqrt
import kotlin.math.tan

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class DrivetrainTest {
    @Test
    fun testDrivetrain() {
        val pursuiter = FastPurePursuit(MockedLocalizer(), Pose2d(0.0,0.0,0.0))
        pursuiter.setStartPoint(Positions.startLeftRed)
        pursuiter.addRelativePoint(-0.5 * SparkAutoBase.TILE_LENGTH, 2 * SparkAutoBase.TILE_LENGTH, 0.0)
        pursuiter.addAction { println("action 1") }



        val goalZone = Pose2d(Positions.bZoneRed, 0.0)

        pursuiter.addPoint(goalZone + Pose2d(-9.0, 0.0))
        pursuiter.addAction { println("action 2") }
        pursuiter.addPoint(Positions.startRightRed + Pose2d(SparkAutoBase.TILE_LENGTH *0.5 + 9.0, Constants.trackwidth * 0.5, PI ))

        val index = 1
        println(pursuiter.waypoints)
        println(pursuiter.actions)
        println(pursuiter.actions.find { it.first == index+1 } == null)
////        val position = Vector2d(0.0, -SparkAutoBase.TILE_LENGTH *1.5)
////        val target = Positions.highGoalRed
////        val targetVector = Vector2d(target.x, target.y)
////        val shootingHeading = targetVector.minus(position).angle()
////        println(shootingHeading)
////        val pose = Pose2d(0.0, 0.0, 0.0)
////        val target = shootingGoal(70.0, 0.0, 35.0)
////        val shooterHeight = 8.0
////        val shooterAngle = Math.toRadians(45.0)
////        val slip = 1.3
////
////        val position = pose.vec()
////        val targetVector = Vector2d(target.x, target.y)
////        val shotDistance = targetVector distTo position
////        val net_height = target.height - shooterHeight
////        val requiredVelocity = Math.sqrt(g /2) * shotDistance/( cos(shooterAngle) * sqrt( shotDistance * tan(shooterAngle) - net_height))
////        println((2*requiredVelocity * slip).toString())
////        println(2*requiredVelocity*slip / (2*2*PI) * 60)
//        val length = 30
//        val lookAhead = 5.0
//
//        val excessLength = (0.8 + (lookAhead / length) - 1.0) * length
//        println(excessLength)



    }
}