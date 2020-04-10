package org.firstinspires.ftc.teamcode.hardware.type

// rev component
open class Device<T>// initialize component
(// component object
        val device: T) {
    companion object {

        // check if double is in specified range
        fun checkRange(value: Double, min: Double, max: Double, varname: String): Double {
            if (value < min || value > max) {
                throw IllegalArgumentException("double $varname out of range: $value min: $min max: $max")
            }
            return value
        }
    }
}