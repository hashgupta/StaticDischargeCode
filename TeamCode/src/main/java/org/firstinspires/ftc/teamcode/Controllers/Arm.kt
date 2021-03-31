package org.firstinspires.ftc.teamcode.Controllers

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.hardware.general.ServoM
import java.lang.Thread.sleep
import kotlin.math.PI

class Arm(val startAngle: Double, val arm_motor: Motor, val grabber: ServoM?) {
    private val reduction = 0.225
    private val autoSpeed = 0.75

    init {
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        arm_motor.setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        arm_motor.device.setVelocityPIDFCoefficients(5.0, 0.0, 0.0, 12.4)
        arm_motor.device.setPositionPIDFCoefficients(5.0)

        arm_motor.device.targetPositionTolerance = 10


    }

    fun toAngle(targetAngle: Double) {
        val revToTurn = (targetAngle - startAngle) / (2 * PI)
        arm_motor.device.targetPosition = (revToTurn * arm_motor.adjusted_tpr).toInt()
        arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION)
        arm_motor.start(autoSpeed)
    }

    fun grabAuto() {


        grabber?.start(1.0)
        sleep(750)


//        toAngle(Math.toRadians(160.0))
//        while ( arm_motor.isBusy) {
//
//        }
//        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
//        run(0.0)
    }

    fun dropAuto() {


//        toAngle(Math.toRadians(90.0))
////        while ( arm_motor.isBusy) {
////
////        }
////        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER)

        run(-autoSpeed)
        sleep(1000)
        run(0.0)
        grabber?.start(0.05)

        sleep(1000)

    }

    fun grabTele() {
        grabber?.start(0.75)
    }

    fun dropTele() {
        grabber?.start(0.05)
    }


    fun run(raw_speed: Double) {
        val adjustedSpeed = raw_speed * reduction
        arm_motor.start(adjustedSpeed)
    }


}