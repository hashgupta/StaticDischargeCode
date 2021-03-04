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

class MecanumLocalizerRev constructor(
        hardwareMap: HardwareMap,
        private val gyro: Gyro?
) : Localizer {
    private var _poseEstimate = Pose2d()

    override var poseEstimate: Pose2d
        get() = _poseEstimate
        set(value) {
            lastWheelPositions = emptyList()
            lastExtHeading = Double.NaN
            gyro?.setExternalHeading(value.heading)
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
        leftFrontEncoder = Encoder(hardwareMap.dcMotor["lf"] as DcMotorEx)
        rightFrontEncoder = Encoder(hardwareMap.dcMotor["rf"] as DcMotorEx)
        leftRearEncoder = Encoder(hardwareMap.dcMotor["lb"] as DcMotorEx)
        rightRearEncoder = Encoder(hardwareMap.dcMotor["rb"] as DcMotorEx)
        leftFrontEncoder.direction = Encoder.Direction.REVERSE
        leftRearEncoder.direction = Encoder.Direction.REVERSE

    }

    fun getWheelPositions(): List<Double> {
        return listOf(
                Constants.encoderTicksToInches(leftFrontEncoder.currentPosition.toDouble()),
                Constants.encoderTicksToInches(leftRearEncoder.currentPosition.toDouble()),
                Constants.encoderTicksToInches(rightRearEncoder.currentPosition.toDouble()),
                Constants.encoderTicksToInches(rightFrontEncoder.currentPosition.toDouble())
        )
    }

    fun getWheelVelocities(): List<Double>? {
        return listOf(
                Constants.encoderTicksToInches(leftFrontEncoder.rawVelocity),
                Constants.encoderTicksToInches(leftRearEncoder.rawVelocity),
                Constants.encoderTicksToInches(rightRearEncoder.rawVelocity),
                Constants.encoderTicksToInches(rightFrontEncoder.rawVelocity)
        )
    }

    fun getExternalHeading(): Double {
        return gyro!!.measure()
    }
    fun getExternalHeadingVelocity() : Double {
        return gyro!!.device.angularVelocity.zRotationRate.toDouble()
    }
}
