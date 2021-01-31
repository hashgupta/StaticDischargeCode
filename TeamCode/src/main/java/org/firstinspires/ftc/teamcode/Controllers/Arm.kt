package org.firstinspires.ftc.teamcode.Controllers

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.hardware.general.ServoM
import java.lang.Thread.sleep
import kotlin.math.PI

class Arm(val startAngle: Double, val arm_motor: Motor, val grabber: ServoM?) {
    val reduction = 0.225
    val auto_speed = 0.3

    init {
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        arm_motor.device.setVelocityPIDFCoefficients(5.0, 0.0, 0.0,12.4)
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


        toAngle(Math.toRadians(160.0))
        while ( arm_motor.isBusy) {

        }
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        run(0.0)
    }

    fun dropAuto() {

        grabber?.start(0.6)
        toAngle(Math.toRadians(45.0))
        while ( arm_motor.isBusy) {

        }
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        run(0.0)

    }


    fun run(raw_speed: Double) {
        val adjusted_speed = raw_speed * reduction
        arm_motor.start(adjusted_speed)
    }


}