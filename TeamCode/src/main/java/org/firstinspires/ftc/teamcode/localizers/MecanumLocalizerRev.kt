package org.firstinspires.ftc.teamcode.localizers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import com.acmerobotics.roadrunner.util.Angle
import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor
import org.firstinspires.ftc.teamcode.hardware.general.Gyro
import org.firstinspires.ftc.teamcode.Constants
import kotlin.math.PI

class MecanumLocalizerRev constructor(
        private val hardwareMap: HardwareMap,
        private val gyro: Gyro?
) : Localizer {
    private var _poseEstimate = Pose2d()
    override var poseEstimate: Pose2d
        get() = _poseEstimate
        set(value) {
            lastWheelPositions = emptyList()
            lastExtHeading = Double.NaN
//            if (useExternalHeading) drive.externalHeading = value.heading
            _poseEstimate = value
        }
    override var poseVelocity: Pose2d? = null
        private set
    private var lastWheelPositions = emptyList<Double>()
    private var lastExtHeading = Double.NaN

    private val leftFrontEncoder: Encoder
    private val rightFrontEncoder: Encoder
    private val leftRearEncoder: Encoder
    private val rightRearEncoder: Encoder

    override fun update() {
        val wheelPositions = getWheelPositions()
        val extHeading = if (gyro != null) getExternalHeading() else Double.NaN
        if (lastWheelPositions.isNotEmpty()) {
            val wheelDeltas = wheelPositions
                    .zip(lastWheelPositions)
                    .map { it.first - it.second }
            val robotPoseDelta = MecanumKinematics.wheelToRobotVelocities(
                    wheelDeltas, Constants.trackwidth, Constants.wheelBase, 1.0
            )
            val finalHeadingDelta = if (gyro != null)
                Angle.normDelta(extHeading - lastExtHeading)
            else
                robotPoseDelta.heading
            _poseEstimate = Kinematics.relativeOdometryUpdate(
                    _poseEstimate,
                    Pose2d(robotPoseDelta.vec(), finalHeadingDelta)
            )
        }

        val wheelVelocities = getWheelVelocities()
        if (wheelVelocities != null) {
            poseVelocity = MecanumKinematics.wheelToRobotVelocities(
                    wheelVelocities, Constants.trackwidth, Constants.wheelBase, 1.0
            )
            if (gyro != null) {
                val extHeadingVel = getExternalHeadingVelocity()
                poseVelocity = Pose2d(poseVelocity!!.vec(), extHeadingVel)
            }
        }

        lastWheelPositions = wheelPositions
        lastExtHeading = extHeading
    }

    init {
        leftFrontEncoder = Encoder(hardwareMap.dcMotor["leftFrontEncoder"] as DcMotorEx)
        rightFrontEncoder = Encoder(hardwareMap.dcMotor["rightFrontEncoder"] as DcMotorEx)
        leftRearEncoder = Encoder(hardwareMap.dcMotor["leftRearEncoder"] as DcMotorEx)
        rightRearEncoder = Encoder(hardwareMap.dcMotor["rightRearEncoder"] as DcMotorEx)
    }

    fun getWheelPositions(): List<Double> {
        return listOf(
                Constants.odometryEncoderTicksToInches(leftFrontEncoder.currentPosition.toDouble()),
                Constants.odometryEncoderTicksToInches(leftRearEncoder.currentPosition.toDouble()),
                Constants.odometryEncoderTicksToInches(rightRearEncoder.currentPosition.toDouble()),
                Constants.odometryEncoderTicksToInches(rightFrontEncoder.currentPosition.toDouble())
        )
    }

    fun getWheelVelocities(): List<Double>? {
        return listOf(
                Constants.odometryEncoderTicksToInches(leftFrontEncoder.rawVelocity),
                Constants.odometryEncoderTicksToInches(leftRearEncoder.rawVelocity),
                Constants.odometryEncoderTicksToInches(rightRearEncoder.rawVelocity),
                Constants.odometryEncoderTicksToInches(rightFrontEncoder.rawVelocity)

        )
    }

    fun getExternalHeading(): Double {
        return gyro!!.measure() * 2 * PI
    }
    fun getExternalHeadingVelocity() : Double {
        return gyro!!.device.angularVelocity.zRotationRate.toDouble()
    }
}
