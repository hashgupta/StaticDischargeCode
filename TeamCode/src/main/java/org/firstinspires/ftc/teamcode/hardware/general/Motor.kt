package org.firstinspires.ftc.teamcode.hardware.general

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.hardware.type.Device
import org.firstinspires.ftc.teamcode.hardware.type.Input
import org.firstinspires.ftc.teamcode.hardware.type.Output
import kotlin.math.PI


// rev motor
// initialize motor
class Motor(private val name: String, // motor information
  val tpr: Double, private val gr: Double, d: Double, map: HardwareMap) : Device<DcMotorEx>(map.dcMotor.get(name) as DcMotorEx), Input<Double>, Output<Double> {

    private val c: Double

    val r: Double
    val isBusy: Boolean
        get() = device.isBusy

    val adjusted_tpr: Double = tpr / gr

    init {
        device.targetPositionTolerance = 10 // (ticks / cpr) * (circumference * gear ratio) is inches of error from tick tolerance
        val motorConfigurationType: MotorConfigurationType = device.motorType.clone()

        motorConfigurationType.achieveableMaxRPMFraction = 1.0

        device.motorType = motorConfigurationType




        c = d * Math.PI
        r = d / 2
    }

    // initialize motor with default diameter
    constructor(name: String, tpr: Double, gr: Double, map: HardwareMap) : this(name, tpr, gr, 1 / Math.PI, map)

    // initialize motor with default gear ratio
    constructor(name: String, tpr: Double, map: HardwareMap) : this(name, tpr, 1.0, map)

    // sense position
    override fun measure(): Double {
        return (device.currentPosition * c * gr) /  tpr
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

    fun setSpeed(velocity:Double, telemetry: Telemetry) { // in/s
//        setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        var angularVelocity = velocity/(r*gr) //radians/s
        val angularVelocityRevS = angularVelocity / (2*PI) // rev/s
        angularVelocity = angularVelocityRevS * tpr //ticks/s

        val dashboard = FtcDashboard.getInstance()
        val DashBoardtelemetry = dashboard.telemetry
        telemetry.addData("ticks/s speed", angularVelocity)
        telemetry.addData("actual ticks/s", device.velocity)
        DashBoardtelemetry.addData("ticks/s speed", angularVelocity)
        DashBoardtelemetry.addData("actual ticks/s", device.velocity)
        DashBoardtelemetry.update()
        device.velocity = angularVelocity
    }
}