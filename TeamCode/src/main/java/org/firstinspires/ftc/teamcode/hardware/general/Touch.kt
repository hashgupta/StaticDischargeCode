package org.firstinspires.ftc.teamcode.hardware.general

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.hardware.type.Device
import org.firstinspires.ftc.teamcode.hardware.type.Input

class Touch(name: String, map: HardwareMap) : Device<TouchSensor>(map.get(TouchSensor::class.java, name)), Input<Boolean> {

    // sense touch
    override fun measure(): Boolean {
        return device.isPressed
    }
}