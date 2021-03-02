package org.firstinspires.ftc.teamcode.purePursuit

import kotlin.math.PI
import kotlin.math.max
import kotlin.math.min

const val TAU = Math.PI * 2

fun lerp(a: Double, b: Double, t: Double): Double {
    return (b - a) * t + a
}

fun lerpAngle(a: Double, b: Double, t: Double): Double {

    return when {
        (b-a) > PI -> {
            lerp(a, b-TAU, t) % TAU
        }
        (b-a) < -PI -> {
            lerp(a, b-TAU, t) % TAU
        }
        else -> {
            lerp(a, b, t) % TAU
        }
    }
}

fun limit(value: Double, min: Double, max: Double): Double {
    return max(min, min(value, max))
}