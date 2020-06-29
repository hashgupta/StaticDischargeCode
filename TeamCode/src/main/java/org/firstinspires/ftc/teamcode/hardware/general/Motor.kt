package org.firstinspires.ftc.teamcode.hardware.general

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.hardware.type.Device
import org.firstinspires.ftc.teamcode.hardware.type.Input
import org.firstinspires.ftc.teamcode.hardware.type.Output

// rev motor
// initialize motor
class Motor(private val name: String, // motor information
 private val tpr: Double, private val gr: Double, d: Double, map: HardwareMap) : Device<DcMotorEx>(map.dcMotor.get(name) as DcMotorEx), Input<Double>, Output<Double> {

    private val c: Double

    val r: Double
    val isBusy: Boolean
        get() = device.isBusy

    init {
        device.targetPositionTolerance = 20 // (ticks / cpr) * (circumference * gear ratio) is inches of error from tick tolerance
        c = d * Math.PI
        r = d / 2
    }

    // initialize motor with default diameter
    constructor(name: String, tpr: Double, gr: Double, map: HardwareMap) : this(name, tpr, gr, 1 / Math.PI, map)

    // initialize motor with default gear ratio
    constructor(name: String, tpr: Double, map: HardwareMap) : this(name, tpr, 1.0, map)

    // sense position
    override fun measure(): Double {
        return device.currentPosition / tpr * gr * c
    }

    // start motion
    override fun start(motion: Double) {
        device.power = checkRange(motion, -1.0, 1.0, this.name)
    }

    fun setTarget(inches: Double) {
        device.targetPosition = (inches * tpr / (gr * c)).toInt() + device.currentPosition
    }

    fun setMode(mode: DcMotor.RunMode) {
        device.mode = mode
    }

    fun setZeroBehavior(behavior: DcMotor.ZeroPowerBehavior) {
        device.zeroPowerBehavior = behavior
    }
}