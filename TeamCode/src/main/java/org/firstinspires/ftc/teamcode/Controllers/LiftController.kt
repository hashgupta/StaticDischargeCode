package org.firstinspires.ftc.teamcode.Controllers

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import kotlin.math.abs

class LiftController (val actuator: Motor, pid: PIDCoefficients, start: Double = 0.0, val maxVel: Double, val maxAcc: Double){
    var goal: Double = 0.0
    private var controller = PIDFController(pid)
    var current: Double = start
    var profile: MotionProfile = MotionProfileGenerator.generateSimpleMotionProfile(MotionState(0.0, 0.0), MotionState(0.0,0.0), 0.0, 0.0)
    val timer = ElapsedTime()

    init {
        controller.setOutputBounds(-1.0, 1.0)
        controller.update(0.0, 0.0)
        controller.targetPosition = 0.0
        controller.targetVelocity = 0.0
        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
    }

    private fun getPosition(): Double {
        current = actuator.measure()
        return current
    }

    private fun getVelocity() : Double {
        return actuator.device.getVelocity(AngleUnit.RADIANS) * actuator.r
    }
    fun to(goal: Double): LiftController {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(MotionState(getPosition(), getVelocity()), MotionState(goal,0.0), maxVel, maxAcc)
        this.goal = goal
        timer.reset()
        return this
    }

    private fun correctionVelocity(): Double {
        val desiredState = profile[timer.seconds()]
        controller.targetPosition = desiredState.x
        controller.targetVelocity = desiredState.v
        return controller.update(getPosition(),getVelocity())
    }

    fun runToTarget() {
        actuator.start(correctionVelocity())
    }

    fun runAtSpeed(speed:Double) {
        actuator.start(speed)
    }

    fun isDone() : Boolean {
        return abs(current - goal) < 0.5
    }
}