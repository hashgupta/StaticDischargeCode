package org.firstinspires.ftc.teamcode.hardware.general

import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.hardware.type.Device
import org.firstinspires.ftc.teamcode.hardware.type.Input

// rev distance sensor
class Distance// initialize sensor
(name: String, // offset from measuring distance
 private val offset: Double, map: HardwareMap) : Device<DistanceSensor>(map.get(DistanceSensor::class.java, name)), Input<Double> {

    // initialize sensor with default offset
    constructor(name: String, map: HardwareMap) : this(name, 0.0, map)

    // sense distance
    override fun measure(): Double {
        return device.getDistance(DistanceUnit.INCH) - offset
    }
}