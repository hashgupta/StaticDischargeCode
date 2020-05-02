package org.firstinspires.ftc.teamcode.purePursuit

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.*

@Config
class ThreeWheelRevLocalizer(hardwareMap: HardwareMap) : ThreeTrackingWheelLocalizer(Arrays.asList(
        Pose2d(0.0, LATERAL_DISTANCE / 2, 0.0),  // lateral left
        Pose2d(FORWARD_OFFSET, 0.0, Math.toRadians(90.0)), // front
        Pose2d(0.0, -LATERAL_DISTANCE / 2, Math.toRadians(180.0)))) { // lateral right

    private val leftEncoder: DcMotor
    private val frontEncoder: DcMotor
    private val rightEncoder: DcMotor

    override fun getWheelPositions(): List<Double> {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.currentPosition),
                encoderTicksToInches(frontEncoder.currentPosition),
                encoderTicksToInches(rightEncoder.currentPosition)
        )
    }

    companion object {
        var TICKS_PER_REV = 0.0
        var WHEEL_RADIUS = 2.0 // in
        var GEAR_RATIO = 1.0 // output (wheel) speed / input (encoder) speed
        var LATERAL_DISTANCE = 10.0 // in; distance between the left and right wheels
        var FORWARD_OFFSET = 4.0 // in; offset of the lateral wheel
        fun encoderTicksToInches(ticks: Int): Double {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
        }
    }

    init {
        leftEncoder = hardwareMap.dcMotor["leftEncoder"]
        frontEncoder = hardwareMap.dcMotor["frontEncoder"]
        rightEncoder = hardwareMap.dcMotor["rightEncoder"]
    }
}