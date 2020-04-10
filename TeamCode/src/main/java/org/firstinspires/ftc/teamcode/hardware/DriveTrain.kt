package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.hardware.type.Input
import org.firstinspires.ftc.teamcode.hardware.type.Output

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

    fun setTarget(inches: Square<Double>) {
        rf.setTarget(inches.rf)
        rb.setTarget(inches.rb)
        lf.setTarget(inches.lf)
        lb.setTarget(inches.lb)
    }

    fun setMode(mode: DcMotor.RunMode) {
        rf.setMode(mode)
        rb.setMode(mode)
        lf.setMode(mode)
        lb.setMode(mode)
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
                    -hori - vert - turn,
                    +hori - vert - turn,
                    -hori + vert - turn,
                    +hori + vert - turn)
        }
    }

    class Vector(hori: Double, vert: Double, turn: Double): Direction(hori, vert, turn) {
        // initialize speeds
        // Device.checkRange(hori, -1, 1, "hori"),
        // Device.checkRange(vert, -1, 1, "vert"),
        // Device.checkRange(turn, -1, 1, "turn"));

        override fun speeds(): Square<Double> {
            val supers = super.speeds()
            var divisor = Math.max(
                    Math.max(
                            Math.abs(supers.rf),
                            Math.abs(supers.rb)
                    ),
                    Math.max(
                            Math.abs(supers.lf),
                            Math.abs(supers.lb)
                    )
            )
            divisor = Math.max(1.0, divisor)
            return Square(
                    supers.rf / divisor,
                    supers.rb / divisor,
                    supers.lf / divisor,
                    supers.lb / divisor)
        }
    }

    // set of data on four objects arranged in a square
    class Square<T>// initialize square
    (// square
            var rf: T, var rb: T, var lf: T, var lb: T)
}