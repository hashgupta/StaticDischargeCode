package org.firstinspires.ftc.teamcode.localizers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.util.Angle
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.hardware.general.Gyro
import kotlin.math.PI

class MockedLocalizer constructor() : Localizer {
    private var _poseEstimate = Pose2d()
    override var poseEstimate: Pose2d
        get() = _poseEstimate
        set(value) {
            _poseEstimate = value
        }
    override var poseVelocity: Pose2d? = null
        private set



    override fun update() {

    }

    init {


    }

//    fun getWheelPositions(): List<Double> {
//
//    }
//
//    fun getWheelVelocities(): List<Double>? {
//
//    }

//    fun getExternalHeading(): Double {
//        return gyro!!.measure() * 2 * PI
//    }
//    fun getExternalHeadingVelocity() : Double {
//        return gyro!!.device.angularVelocity.zRotationRate.toDouble()
//    }
}