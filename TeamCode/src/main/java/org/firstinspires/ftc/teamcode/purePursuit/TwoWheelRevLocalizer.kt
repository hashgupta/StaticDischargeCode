package org.firstinspires.ftc.teamcode.purePursuit

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.purePursuit.Constants.FORWARD_OFFSET
import org.firstinspires.ftc.teamcode.purePursuit.Constants.LATERAL_DISTANCE
import org.firstinspires.ftc.teamcode.purePursuit.Constants.encoderTicksToInches

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
    private val lateralEncoder: DcMotor
    private val frontEncoder: DcMotor
    private val imuSensor: BNO055IMU

    override fun getWheelPositions(): List<Double> {
        return listOf(
                encoderTicksToInches(lateralEncoder.currentPosition),
                encoderTicksToInches(frontEncoder.currentPosition)
        )
    }

    override fun getHeading(): Double {
        return imuSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle.toDouble()
    }


    init {
        lateralEncoder = hardwareMap.dcMotor["lateralEncoder"]
        frontEncoder = hardwareMap.dcMotor["frontEncoder"]
        val parameters = BNO055IMU.Parameters()
        parameters.mode = BNO055IMU.SensorMode.IMU
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.loggingEnabled = false
        imuSensor = hardwareMap.get(BNO055IMU::class.java, "imu")
    }
}