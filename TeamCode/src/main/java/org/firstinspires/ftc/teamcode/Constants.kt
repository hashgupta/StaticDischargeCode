package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config

@Config
object Constants {

    const val trackwidth = 12.0
    const val wheelBase = 8.25
    const val robotWidth = 18.0 // this is from left side to right side
    const val robotLength = 18.0 // front to back

    const val TICKS_PER_REV = 1120.0
    const val WHEEL_DIAMETER = 2.95 // in
    const val GEAR_RATIO = 1.0/1.0 // output (wheel) speed / input (encoder) speed


    const val LATERAL_DISTANCE = 8.0 // in; distance between the left odometry wheel and center
    const val FORWARD_OFFSET = -3.6 // in; offset of the front/back wheel from center
    const val ODO_WHEEL_DIAM = 2.3622
    const val ODO_TICKS_PER_REV = 8192

    fun encoderTicksToInches(ticks: Double): Double {
        return WHEEL_DIAMETER * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
    }
    fun odometryEncoderTicksToInches(ticks: Double): Double {
        return ODO_WHEEL_DIAM * Math.PI * ticks / ODO_TICKS_PER_REV
    }

}