package org.firstinspires.ftc.teamcode.Controllers

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.hardware.general.ServoM
import java.lang.Thread.sleep
import kotlin.math.PI

class Arm(val startAngle: Double, val arm_motor: Motor, val grabber: ServoM) {
    init {
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        arm_motor.device.targetPositionTolerance = 1
    }
    fun toAngle(targetAngle:Double) {
        val revToTurn = -(targetAngle - startAngle) / (2* PI)
        arm_motor.device.targetPosition = (revToTurn * arm_motor.tpr).toInt()
        arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION)
        arm_motor.start(0.7)
    }

    fun grabAuto() {
//        toAngle(Math.toRadians(0.0))
//        while ( arm_motor.isBusy) {
//
//        }
//        run(0.0)

        grabber.start(0.0)
        sleep(500)
//        grabber.start(1.0)
        toAngle(Math.toRadians(90.0))
        while ( arm_motor.isBusy) {

        }
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        run(0.0)
    }

    fun dropAuto() {

        grabber.start(0.6)
        sleep(500)

    }

    fun grabTele() {
        grabber.start(0.0)
    }

//    fun stopGrabber() {
//        grabber.start(0.5)
//    }

    fun dropTele() {
        grabber.start(0.3)
    }


    fun run(speed: Double) {
        var adjusted_speed = speed * 0.5
        arm_motor.start(adjusted_speed)
    }


}