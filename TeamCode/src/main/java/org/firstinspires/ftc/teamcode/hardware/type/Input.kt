package org.firstinspires.ftc.teamcode.hardware.type

// sensing component
interface Input<T> {
    // sense component
    fun measure(): T
}