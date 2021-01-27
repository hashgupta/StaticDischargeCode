package org.firstinspires.ftc.teamcode.hardware.general

import com.acmerobotics.roadrunner.util.Angle
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.hardware.type.Device
import org.firstinspires.ftc.teamcode.hardware.type.Input
import kotlin.math.PI

// rev expansion hub used as a gyro sensor
class Gyro// initialize sensor
(private val name: String, map: HardwareMap) : Device<BNO055IMU>(map.get(BNO055IMU::class.java, name)), Input<Double> {
    private var headingOffset: Double

    init {
        val params = BNO055IMU.Parameters()
        params.mode = BNO055IMU.SensorMode.IMU
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS
        device.initialize(params)
        headingOffset = 0.0
        while (!device.isGyroCalibrated) {
        }
    }

    // sense angle
    override fun measure(): Double {
        val angles = device.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
        return checkRange((angles.firstAngle / 360).toDouble(), -0.5, 0.5, name)
    }

    fun measureRadians(): Double {
        val angles = device.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS)
        return Angle.norm(angles.firstAngle.toDouble() + headingOffset)
    }

    fun setExternalHeading(value: Double) {
        headingOffset = value - measureRadians()

    }
}