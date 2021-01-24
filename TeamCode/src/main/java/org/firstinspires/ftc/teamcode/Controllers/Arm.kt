package org.firstinspires.ftc.teamcode.Controllers

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.hardware.general.ServoM
import java.lang.Thread.sleep
import kotlin.math.PI

class Arm(val startAngle: Double, val arm_motor: Motor, val grabber: ServoM?) {
    val reduction = 0.125
    val auto_speed = 0.2

    init {
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        arm_motor.device.setVelocityPIDFCoefficients(1.26, 0.85, 0.0,12.6)
        arm_motor.device.setPositionPIDFCoefficients(5.0)

        arm_motor.setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE)

        arm_motor.device.targetPositionTolerance = 10
    }

    fun toAngle(targetAngle:Double) {
        val revToTurn = -(targetAngle - startAngle) / (2* PI)
        arm_motor.device.targetPosition = (revToTurn * arm_motor.adjusted_tpr).toInt()
        arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION)
        arm_motor.start(auto_speed)
    }

    fun grabAuto() {


        grabber?.start(0.0)
//        sleep(500)

        toAngle(Math.toRadians(90.0))
        while ( arm_motor.isBusy) {

        }
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        run(0.0)
    }

    fun dropAuto() {

        grabber?.start(0.6)
        sleep(500)

    }

    fun grabTele() {
        grabber?.start(0.0)
    }

//    fun stopGrabber() {
//        grabber.start(0.5)
//    }

    fun dropTele() {
        grabber?.start(0.3)
    }


    fun run(raw_speed: Double) {
        val adjusted_speed = raw_speed * reduction
        arm_motor.start(adjusted_speed)
    }


}