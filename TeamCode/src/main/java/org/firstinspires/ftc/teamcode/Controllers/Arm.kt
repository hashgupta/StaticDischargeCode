package org.firstinspires.ftc.teamcode.Controllers

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.hardware.general.ServoCRWrapper
import java.lang.Thread.sleep
import kotlin.math.PI

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
        grabber.start(0.5)
    }

    fun dropTele() {
        grabber.start(0.0)
    }


    fun run(speed: Double) {
        var adjusted_speed = speed * 0.75
        arm_motor.start(adjusted_speed)
    }
}