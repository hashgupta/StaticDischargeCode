package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config

@Config
object Constants {
//    const val m = 15
//    const val g = 9.8
//    const val mu = 0.6
//    const val maxMotorAmps = 20
//    const val motorVolts = 3.3
    const val trackwidth = 18.0
    const val wheelBase = 18.0
//    const val maxMotorPower = maxMotorAmps * motorVolts
//    const val numOfWheels = 4 // put in to scale the power to distribute it over the wheels
//    const val WheelVelToPowerConst =(m*g*mu / numOfWheels) / maxMotorPower
    const val TICKS_PER_REV = 1120.0
    const val WHEEL_DIAMETER = 2.95 // in
    const val GEAR_RATIO = 1.0/1.0 // output (wheel) speed / input (encoder) speed
    const val LATERAL_DISTANCE = 10.0 // in; distance between the left and right wheels (ie trackwidth)
    const val FORWARD_OFFSET = 4.0 // in; offset of the front/back wheel
    const val ODO_WHEEL_RADIUS = 1.0

    fun encoderTicksToInches(ticks: Double): Double {
        return WHEEL_DIAMETER * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
    }
    fun odometryEncoderTicksToInches(ticks: Double): Double {
        return ODO_WHEEL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV
    }
    // formula is power = force * velocity
    // what we want to do is get a net power of zero, so the velocity of the body is at the desired vel
    // so the power supplied has to equal the power lost to friction
    // power lost equals the force (friction) times the current velocity
    // therefore if we can calculate the power lost, we can supply that much to the wheels to make the net power zero
    // the best part is that no matter how fast the body is moving, if we set the power to equal what would be lost
    // at the desired velocity, the body will automatically adjust to that desired

}