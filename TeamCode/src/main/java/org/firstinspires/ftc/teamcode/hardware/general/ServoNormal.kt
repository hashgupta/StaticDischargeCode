package org.firstinspires.ftc.teamcode.hardware.general

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

import org.firstinspires.ftc.teamcode.hardware.type.Device
import org.firstinspires.ftc.teamcode.hardware.type.Input
import org.firstinspires.ftc.teamcode.hardware.type.Output

// rev servo
class ServoNormal// initialize servo
(private val name: String, map: HardwareMap) : Device<Servo>(map.servo.get(name)), Input<Double>, Output<Double> {

    // sense position
    override fun measure(): Double {
        return device.position
    }

    // start motion
    override fun start(motion: Double) {
        device.position = checkRange(motion, 0.0, 1.0, this.name)
    }
}