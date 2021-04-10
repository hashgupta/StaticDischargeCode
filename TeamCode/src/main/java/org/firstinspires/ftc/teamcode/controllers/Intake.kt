package org.firstinspires.ftc.teamcode.controllers

import org.firstinspires.ftc.teamcode.hardware.general.Motor

// Holonomic Mecanum/quad-omni drive train
class Intake(val motors: Array<Motor>, val speeds: Array<Double>) {

    enum class Mode {
        Forward,
        Backward,
        OFF
    }

    var currentMode = Mode.OFF


    fun run() {
        when (currentMode) {
            Mode.Forward -> {
                motors.zip(speeds).forEach { it.first.start(it.second) }
            }
            Mode.Backward -> {
                motors.zip(speeds).forEach { it.first.start(-it.second) }
            }
            else -> {
                motors.forEach { it.start(0.0) }
            }
        }
    }

    fun setForward() {
        currentMode = Mode.Forward
    }

    fun setBackward() {
        currentMode = Mode.Backward
    }


}