package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.Controllers.shootingGoal

const val tile_length = 24.0

object Positions {

    val highGoalRed = shootingGoal(3 * tile_length, -1.5* tile_length, 35.0)
    val highGoalBlue = shootingGoal(3 * tile_length, 1.5* tile_length, 35.0)

    val midGoalRed = shootingGoal(3 * tile_length, -1.5* tile_length, 28.0)
    val midGoalBlue = shootingGoal(3 * tile_length, 1.5* tile_length, 28.0)

    val aZoneBlue = Vector2d(0.5* tile_length, 2.5* tile_length)
    val bZoneBlue = Vector2d(1.5* tile_length, 1.5* tile_length)
    val cZoneBlue = Vector2d(2.5* tile_length, 2.5* tile_length)

    val aZoneRed = Vector2d(0.5* tile_length, -2.5* tile_length)
    val bZoneRed = Vector2d(1.5* tile_length, -1.5* tile_length)
    val cZoneRed = Vector2d(2.5* tile_length, -2.5* tile_length)

    val startLeftRed = Pose2d(-3* tile_length + Constants.robotLength * 0.5, -tile_length - Constants.robotLength * 0.5, 0.0)
    val startRightRed = Pose2d(-3* tile_length + Constants.robotLength * 0.5, -2* tile_length, 0.0)

    val startLeftBlue = Pose2d(-3* tile_length + Constants.robotLength * 0.5, 2* tile_length, 0.0)
    val startRightBlue = Pose2d(-3* tile_length + Constants.robotLength * 0.5, tile_length, 0.0)

    val powerNearBlue = shootingGoal(3 * tile_length, 0.75* tile_length, 32.0)
    val powerMidBlue = shootingGoal(3 * tile_length, 0.5* tile_length, 32.0)
    val powerFarBlue = shootingGoal(3 * tile_length, 0.25* tile_length, 32.0)

    val powerNearRed = shootingGoal(3 * tile_length, -0.75* tile_length, 32.0)
    val powerMidRed = shootingGoal(3 * tile_length, -0.5* tile_length, 32.0)
    val powerFarRed = shootingGoal(3 * tile_length, -0.25* tile_length, 32.0)
}