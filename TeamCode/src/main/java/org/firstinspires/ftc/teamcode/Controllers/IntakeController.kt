package org.firstinspires.ftc.teamcode.Controllers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.hardware.type.Input
import org.firstinspires.ftc.teamcode.hardware.type.Output
import kotlin.math.abs
import kotlin.math.max

// Holonomic Mecanum/quad-omni drive train
class IntakeController(val motors: Array<Motor>, val speeds: Array<Double>) {

    enum class Mode {
        Forward,
        Backward,
        OFF
    }

    val currentMode = Mode.OFF


    fun run() {
        if (currentMode == Mode.Forward) {
            motors.zip(speeds).forEach {it.first.start(it.second)}
        } else if (currentMode == Mode.Backward) {
            motors.zip(speeds).forEach {it.first.start(-it.second)}
        } else {
            motors.forEach { it.start(0.0) }
        }
    }



}