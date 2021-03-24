package org.firstinspires.ftc.teamcode.localizers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Constants.FORWARD_OFFSET
import org.firstinspires.ftc.teamcode.Constants.LATERAL_DISTANCE
import org.firstinspires.ftc.teamcode.Constants.odometryEncoderTicksToInches
import org.firstinspires.ftc.teamcode.hardware.general.Gyro

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

class TwoWheelRevLocalizer(hardwareMap: HardwareMap, frontName: String, lateralName: String, gyro: Gyro) : TwoTrackingWheelLocalizer(listOf(
        Pose2d(-2.25, LATERAL_DISTANCE, Math.toRadians(0.0)),  // lateral
        Pose2d(FORWARD_OFFSET, 2.0, Math.toRadians(90.0)))) { //front
    private val lateralEncoder: Encoder = Encoder(hardwareMap.dcMotor[lateralName] as DcMotorEx)
    private val frontEncoder: Encoder = Encoder(hardwareMap.dcMotor[frontName] as DcMotorEx)
    private val imuGyro: Gyro = gyro

    override fun getWheelPositions(): List<Double> {
        return listOf(
                odometryEncoderTicksToInches(lateralEncoder.currentPosition.toDouble() * 1.0169),
                odometryEncoderTicksToInches(frontEncoder.currentPosition.toDouble())
        )
    }

    override fun getHeading(): Double {
        return imuGyro.measure()
    }


}