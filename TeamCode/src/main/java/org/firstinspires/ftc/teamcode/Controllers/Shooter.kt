package org.firstinspires.ftc.teamcode.Controllers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.util.Angle
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.hardware.general.ServoM
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sqrt
import kotlin.math.tan

const val g = 386.088583 //  g in in/s^2

class Shooter(val flywheel: Motor, val shooterAngle:Double, val shooterHeight:Double, val telemetry: Telemetry, val flicker: ServoM? = null){
    var flickerTimingMS = 200.0
    var slip = 1.046 // flywheel shooter slip, MUST BE TUNED
    val turnCorrection = PI

    init {
        //https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.4vkznp7wtsch
        flywheel.device.setVelocityPIDFCoefficients(50.0, 0.0, 0.1,13.6)
        flywheel.device.setPositionPIDFCoefficients(5.0)

        flywheel.setZeroBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
    }

    fun simpleShootAtTarget(pose: Pose2d, target: shootingGoal) {
        //start up flywheel at desired velocity
        //use if using basic move commands without pure pursuit
        val position = pose.vec()
        val targetVector = Vector2d(target.x, target.y)
        val shotDistance = targetVector distTo position
        telemetry.addData("dist",shotDistance)
        val net_height = target.height - shooterHeight
        val requiredVelocity = Math.sqrt(g /2) * shotDistance/( cos(shooterAngle) * sqrt( shotDistance * tan(shooterAngle) - net_height))

        flywheel.setSpeed(2*requiredVelocity * slip, telemetry) // remove 2 times if using double flywheel, doesnt account for direction

        telemetry.update()

    }

    fun turningTarget(position: Vector2d, target: shootingGoal): Double {
        val targetVector = Vector2d(target.x, target.y)
        val shootingHeadingVector = targetVector.minus(position)

        return Angle.norm(shootingHeadingVector.angle() + turnCorrection)

    }

    fun shoot() {
        //release chamber servo to let a ring into flywheel
        if (flicker != null) {
            flicker.start(0.50)
            Thread.sleep(flickerTimingMS.toLong())
            flicker.start(0.9)
        }
    //dpad up clockwise
        //dpad down ccw

    }

    fun stopWheel() {
        flywheel.setSpeed(0.0, telemetry)
    }
}


data class shootingGoal(val x:Double, val y:Double, val height:Double)