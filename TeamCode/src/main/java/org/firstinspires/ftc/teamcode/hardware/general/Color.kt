package org.firstinspires.ftc.teamcode.hardware.general

import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.HardwareMap

import org.firstinspires.ftc.teamcode.hardware.type.Device
import org.firstinspires.ftc.teamcode.hardware.type.Input

// rev color sensor
class Color// initialize sensor
(name: String, map: HardwareMap) : Device<ColorSensor>(map.get(ColorSensor::class.java, name)), Input<Color.HSV> {

    // sense color
    override fun measure(): HSV {
        val hsv = FloatArray(3)
        android.graphics.Color.RGBToHSV(
                255 * device.red(),
                255 * device.green(),
                255 * device.blue(),
                hsv)
        return HSV(hsv[0].toDouble() / 360, hsv[1].toDouble() / 100, hsv[2].toDouble() / 100)
    }

    // color value stored in hsv
    class HSV// initialize color
    (h: Double, s: Double, v: Double) {

        // hsv values
        private val h: Double = checkRange(h, 0.0, 1.0, "hue")
        private val s: Double = checkRange(s, 0.0, 1.0, "saturation")
        private val v: Double = checkRange(v, 0.0, 1.0, "value")

        // get values
        fun h(): Double {
            return h
        }

        fun s(): Double {
            return s
        }

        fun v(): Double {
            return v
        }
    }
}