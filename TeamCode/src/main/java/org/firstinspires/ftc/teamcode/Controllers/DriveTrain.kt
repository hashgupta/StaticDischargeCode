package org.firstinspires.ftc.teamcode.Controllers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.hardware.type.Input
import org.firstinspires.ftc.teamcode.hardware.type.Output
import kotlin.math.abs
import kotlin.math.max

// Holonomic Mecanum/quad-omni drive train
class DriveTrain// initialize drive train
(// motors
        private val rf: Motor, private val rb: Motor, private val lf: Motor, private val lb: Motor) : Input<DriveTrain.Square<Double>>, Output<DriveTrain.Square<Double>> {


    val isBusy: Boolean
        get() = rf.isBusy || rb.isBusy || lf.isBusy || lb.isBusy

    // sense position
    override fun measure(): Square<Double> {
        return Square(rf.measure(), rb.measure(), lf.measure(), lb.measure())
    }

    // start motion
    override fun start(motion: Square<Double>) {
        rf.start(motion.rf)
        rb.start(motion.rb)
        lf.start(motion.lf)
        lb.start(motion.lb)
    }

    fun startFromRRPower(velocity: Pose2d) {
        val vert = velocity.x
        val hori = -velocity.y

        val turn = -velocity.heading
        val square = Vector(hori, vert, turn).speeds()
        start(square)
    }

    fun setTarget(inches: Square<Double>) {
        rf.setTarget(inches.rf)
        rb.setTarget(inches.rb)
        lf.setTarget(inches.lf)
        lb.setTarget(inches.lb)
    }
    fun getPosition():Square<Int> {
        return Square<Int>(rf.device.currentPosition, rb.device.currentPosition, lf.device.currentPosition, lb.device.currentPosition)
    }

    fun setMode(mode: DcMotor.RunMode) {
        rf.setMode(mode)
        rb.setMode(mode)
        lf.setMode(mode)
        lb.setMode(mode)
    }

    fun getTarget():Square<Int> {
        return Square<Int>(rf.device.targetPosition,rb.device.targetPosition, lf.device.targetPosition, lb.device.targetPosition)
    }

    fun setZeroBehavior(behavior: DcMotor.ZeroPowerBehavior) {
        rf.setZeroBehavior(behavior)
        rb.setZeroBehavior(behavior)
        lf.setZeroBehavior(behavior)
        lb.setZeroBehavior(behavior)
    }

    // vector for x, y, and turn
    open class Direction// initialize distances
    (// distances
            val hori: Double, val vert: Double, val turn: Double) {

        // get wheel distances
        open fun speeds(): Square<Double> {
            return Square(
                    -hori + vert - turn,
                    +hori + vert - turn,
                    -hori - vert - turn,
                    +hori - vert - turn)
        }
    }

    class Vector(hori: Double, vert: Double, turn: Double): Direction(hori, vert, turn) {
        // initialize speeds
//         Device.checkRange(hori, -1, 1, "hori"),
//         Device.checkRange(vert, -1, 1, "vert"),
//         Device.checkRange(turn, -1, 1, "turn"))

        override fun speeds(): Square<Double> {
            val supers = super.speeds()
            var divisor = max(
                    max(
                            abs(supers.rf),
                            abs(supers.rb)
                    ),
                    max(
                            abs(supers.lf),
                            abs(supers.lb)
                    )
            )
            divisor = max(1.0, divisor)
            return Square(
                    supers.rf / divisor,
                    supers.rb / divisor,
                    supers.lf / divisor,
                    supers.lb / divisor)
        }
    }

    // set of data on four objects arranged in a square
    // square
    data class Square<T> (var rf: T, var rb: T, var lf: T, var lb: T) {
        override fun toString(): String {
            return "rf: "+rf+", rb: "+rb+", lf: "+ lf +", lb: " + lb
        }
    }

    companion object {
        fun addSquares(first: Square<Double>, second: Square<Double>): Square<Double> {
            return Square(first.rf + second.rf, first.rb + second.rb, first.lf + second.lf, first.lb + second.lb)
        }
    }
}