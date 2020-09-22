package org.firstinspires.ftc.teamcode.localizers

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.Constants.FORWARD_OFFSET
import org.firstinspires.ftc.teamcode.Constants.LATERAL_DISTANCE
import org.firstinspires.ftc.teamcode.Constants.encoderTicksToInches
import org.firstinspires.ftc.teamcode.Constants.odometryEncoderTicksToInches

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *         front
 *    /--------------\
 *    |     ____     |          ^ positive x
 *    |     ----     |          < positive y
 *    | ||           |
 *    | ||           |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Note: this could be optimized significantly with REV bulk reads
 */
@Config
class TwoWheelRevLocalizer(hardwareMap: HardwareMap) : TwoTrackingWheelLocalizer(listOf(
        Pose2d(0.0, LATERAL_DISTANCE / 2, 0.0),  // lateral
        Pose2d(FORWARD_OFFSET, 0.0, Math.toRadians(90.0)) )) { //front
    private val lateralEncoder: Encoder
    private val frontEncoder: Encoder
    private val imuSensor: BNO055IMU

    override fun getWheelPositions(): List<Double> {
        return listOf(
                odometryEncoderTicksToInches(lateralEncoder.currentPosition.toDouble()),
                odometryEncoderTicksToInches(frontEncoder.currentPosition.toDouble())
        )
    }

    override fun getWheelVelocities(): List<Double>? {
        return listOf(
                odometryEncoderTicksToInches(lateralEncoder.correctedVelocity),
                odometryEncoderTicksToInches(frontEncoder.correctedVelocity)
        )
    }

    override fun getHeading(): Double {
        return imuSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle.toDouble()
    }


    init {
        lateralEncoder = Encoder(hardwareMap.dcMotor["lateralEncoder"] as DcMotorEx)
        frontEncoder = Encoder(hardwareMap.dcMotor["frontEncoder"] as DcMotorEx)
        imuSensor = hardwareMap.get(BNO055IMU::class.java, "imu")
    }
}