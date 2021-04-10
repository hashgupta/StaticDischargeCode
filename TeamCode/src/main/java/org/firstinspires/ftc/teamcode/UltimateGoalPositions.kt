package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.controllers.ShootingGoal

const val tile_length = 24.0

object UltimateGoalPositions {

    val highGoalRed = ShootingGoal(3 * tile_length, -1.5 * tile_length, 36.0)
    val highGoalBlue = ShootingGoal(3 * tile_length, 1.5 * tile_length, 36.0)

    val midGoalRed = ShootingGoal(3 * tile_length, -1.5 * tile_length, 28.0)
    val midGoalBlue = ShootingGoal(3 * tile_length, 1.5 * tile_length, 28.0)

    val aZoneBlue = Vector2d(0.5 * tile_length, 2.5 * tile_length)
    val bZoneBlue = Vector2d(1.5 * tile_length, 1.5 * tile_length)
    val cZoneBlue = Vector2d(2.5 * tile_length, 2.5 * tile_length)

    val aZoneRed = Vector2d(0.5 * tile_length, -2.5 * tile_length)
    val bZoneRed = Vector2d(1.5 * tile_length, -1.5 * tile_length)
    val cZoneRed = Vector2d(2.5 * tile_length, -2.5 * tile_length)

    val startLeftRed = Pose2d(-3 * tile_length + Constants.robotLength * 0.5, -tile_length - Constants.robotLength * 0.5, 0.0)
    val startRightRed = Pose2d(-3 * tile_length + Constants.robotLength * 0.5, -2 * tile_length, 0.0)

    val startLeftBlue = Pose2d(-3 * tile_length + Constants.robotLength * 0.5, 2 * tile_length, 0.0)
    val startRightBlue = Pose2d(-3 * tile_length + Constants.robotLength * 0.5, tile_length, 0.0)

    val powerNearBlue = ShootingGoal(3 * tile_length, 0.75 * tile_length, 31.0)
    val powerMidBlue = ShootingGoal(3 * tile_length, 0.5 * tile_length, 31.0)
    val powerFarBlue = ShootingGoal(3 * tile_length, 0.25 * tile_length, 31.0)

    val powerNearRed = ShootingGoal(3 * tile_length, -0.75 * tile_length, 33.0)
    val powerMidRed = ShootingGoal(3 * tile_length, -0.5 * tile_length, 33.0)
    val powerFarRed = ShootingGoal(3 * tile_length, -0.25 * tile_length, 33.0)
}