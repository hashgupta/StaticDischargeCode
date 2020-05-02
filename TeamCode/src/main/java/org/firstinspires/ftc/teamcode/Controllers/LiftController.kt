package org.firstinspires.ftc.teamcode.Controllers

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import org.firstinspires.ftc.teamcode.hardware.general.Motor

class LiftController (val actuator: Motor, pid: PIDCoefficients, start: Double = 0.0){
    var goal: Double = 0.0;
    private var controller = PIDFController(pid)
    var current: Double = start

    init {
        controller.setOutputBounds(-1.0, 1.0)
    }

    private fun getPosition(): Double {
        return actuator.measure()
    }
    fun to(goal: Double): LiftController {
        this.goal = goal
        controller.targetPosition = goal
        return this
    }

    private fun correctionVelocity(): Double {
        current = getPosition()
        return controller.update(current)
    }

    fun run() {
        actuator.start(correctionVelocity())
    }

}