package org.firstinspires.ftc.teamcode.Controllers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.StaticSparky.SparkyRobot
import org.firstinspires.ftc.teamcode.hardware.general.Motor

const val g = 386.088583 //  g in in/s^2

class Shooter(val flywheel: Motor){
    fun startShootAtTarget(robot: SparkyRobot, target: shootingGoal, shooterAngle:Double) {
        //start up flywheel at desired velocity and move robot to correct orientation
        val position = robot.localizer.poseEstimate.vec()
        val targetVector = Vector2d(target.x, target.y)
        val shootingHeading = targetVector.minus(position).angle()
        val shotDistance = targetVector distTo position
        robot.pursuiter.addPoint(Pose2d(robot.localizer.poseEstimate.vec(), shootingHeading))
        robot.pursuiter.FollowSync(robot.driveTrain)
        val requiredVelocity = Math.sqrt(g /2) * shotDistance/( Math.cos(shooterAngle) * Math.sqrt( shotDistance * Math.tan(shooterAngle) - target.height))
        flywheel.setSpeed(2*requiredVelocity) // remove 2 times if using double flywheel, doesnt account for direction

    }

    fun shoot() {
        //release chamber servo to let a ring into flywheel

    }
}


data class shootingGoal(val x:Double, val y:Double, val height:Double)