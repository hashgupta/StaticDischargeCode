package org.firstinspires.ftc.teamcode.hardware.general

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.hardware.type.Device
import org.firstinspires.ftc.teamcode.hardware.type.Input
import org.firstinspires.ftc.teamcode.purepursuit.limit

class RevPotentiometer
(name: String, map: HardwareMap) : Device<AnalogInput>(map.get(AnalogInput::class.java, name)), Input<Double> {

    // outputs in degrees
    override fun measure(): Double {
        val v = device.voltage
        fun equation(degrees: Double):Double {
            return v - (445.5 * (degrees - 270) / (degrees*degrees - 270 * degrees - 36450))
        }

        var estimate = 135.0
        val h = 0.001

        for (i in 0..5) {
            val output = equation(estimate)
            val nudged = equation(estimate + h)
            estimate = estimate - output * h / (nudged - output)
            estimate = limit(estimate, 0.0, 270.0)
        }
        return estimate
    }

    fun getVoltage(): Double {
        return device.voltage
    }




}