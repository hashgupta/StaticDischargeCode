package org.firstinspires.ftc.teamcode.Controllers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.hardware.general.ServoCRWrapper
import org.firstinspires.ftc.teamcode.staticSparky.SparkyRobot
import java.lang.Thread.sleep
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sqrt
import kotlin.math.tan

class Arm(val startAngle: Double, val arm_motor: Motor, val grabber: ServoCRWrapper) {
    init {
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
    }
    fun toAngle(targetAngle:Double) {
        val revToTurn = (targetAngle - startAngle) / (2* PI)
        arm_motor.device.targetPosition = (revToTurn * arm_motor.tpr).toInt()
        arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION)
        arm_motor.start(0.8)
    }

    fun grabAuto() {
        grabber.start(1.0)
        sleep(1000)
        grabber.start(0.1)
    }

    fun dropAuto() {
        grabber.start(-1.0)
        sleep(1000)
    }

    fun grabTele() {
        grabber.start(1.0)
    }

    fun stopGrabber() {
        grabber.start(0.0)
    }

    fun dropTele() {
        grabber.start(-1.0)
    }


    fun run(speed: Double) {
        arm_motor.start(speed)
    }
}