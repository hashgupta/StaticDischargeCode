package org.firstinspires.ftc.teamcode.purePursuit

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.purePursuit.Constants.FORWARD_OFFSET
import org.firstinspires.ftc.teamcode.purePursuit.Constants.LATERAL_DISTANCE
import org.firstinspires.ftc.teamcode.purePursuit.Constants.encoderTicksToInches

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
                encoderTicksToInches(leftEncoder.currentPosition),
                encoderTicksToInches(frontEncoder.currentPosition),
                encoderTicksToInches(rightEncoder.currentPosition)
        )
    }

    override fun getWheelVelocities(): List<Double>? {
        return listOf(
                leftEncoder.getVelocity(AngleUnit.RADIANS) * Constants.ODO_WHEEL_RADIUS,
                frontEncoder.getVelocity(AngleUnit.RADIANS) * Constants.ODO_WHEEL_RADIUS,
                rightEncoder.getVelocity(AngleUnit.RADIANS) * Constants.ODO_WHEEL_RADIUS
        )
    }



    init {
        leftEncoder = hardwareMap.dcMotor["leftEncoder"] as DcMotorEx
        frontEncoder = hardwareMap.dcMotor["frontEncoder"] as DcMotorEx
        rightEncoder = hardwareMap.dcMotor["rightEncoder"] as DcMotorEx
    }
}