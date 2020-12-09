package org.firstinspires.ftc.teamcode.hardware.general

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap

import org.firstinspires.ftc.teamcode.hardware.type.Device
import org.firstinspires.ftc.teamcode.hardware.type.Input
import org.firstinspires.ftc.teamcode.hardware.type.Output

// rev servo
class ServoCRWrapper// initialize servo
(private val name: String, map: HardwareMap) : Device<CRServo>(map.crservo.get(name)), Input<Double>, Output<Double> {

    // sense position
    override fun measure(): Double {
        return device.power
    }

    // start motion
    override fun start(motion: Double) {
//        device.power = checkRange(motion, -1.0, 1.0, this.name)
        device.setPower(motion)
    }
}