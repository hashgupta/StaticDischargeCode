package org.firstinspires.ftc.teamcode.localizers

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.Constants.FORWARD_OFFSET
import org.firstinspires.ftc.teamcode.Constants.LATERAL_DISTANCE
import org.firstinspires.ftc.teamcode.Constants.encoderTicksToInches
import org.firstinspires.ftc.teamcode.Constants.odometryEncoderTicksToInches

@Config
class ThreeWheelRevLocalizer(hardwareMap: HardwareMap) : ThreeTrackingWheelLocalizer(listOf(
        Pose2d(0.0, LATERAL_DISTANCE / 2, 0.0),  // lateral left
    Pose2d(-FORWARD_OFFSET, 0.0, Math.toRadians(90.0)), // back
    Pose2d(0.0, -LATERAL_DISTANCE / 2, Math.toRadians(180.0)))) { // lateral right

    private val leftEncoder: DcMotorEx
    private val frontEncoder: DcMotorEx
    private val rightEncoder: DcMotorEx

    override fun getWheelPositions(): List<Double> {
        return listOf(
                odometryEncoderTicksToInches(leftEncoder.currentPosition.toDouble()),
                odometryEncoderTicksToInches(frontEncoder.currentPosition.toDouble()),
                odometryEncoderTicksToInches(rightEncoder.currentPosition.toDouble())
        )
    }

    override fun getWheelVelocities(): List<Double>? {
        return listOf(
                odometryEncoderTicksToInches(leftEncoder.getVelocity()),
                odometryEncoderTicksToInches(frontEncoder.getVelocity()),
                odometryEncoderTicksToInches(rightEncoder.getVelocity())
        )
    }



    init {
        leftEncoder = hardwareMap.dcMotor["leftEncoder"] as DcMotorEx
        frontEncoder = hardwareMap.dcMotor["frontEncoder"] as DcMotorEx
        rightEncoder = hardwareMap.dcMotor["rightEncoder"] as DcMotorEx
    }
}