package org.firstinspires.ftc.teamcode.hardware.type

// mechanical component
interface Output<T> {
    // start component
    fun start(motion: T)
}