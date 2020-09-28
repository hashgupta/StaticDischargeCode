package org.firstinspires.ftc.teamcode.StaticSparky

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.firstinspires.ftc.teamcode.hardware.general.Gyro
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.hardware.general.Touch
import org.firstinspires.ftc.teamcode.localizers.MecanumLocalizerRev
import org.firstinspires.ftc.teamcode.purePursuit.FastPurePursuit
import kotlin.math.abs
import kotlin.math.sign

class TestRobot (val hardwareMap: HardwareMap, val telemetry: Telemetry) {
    // OpMode members
    val driveTrain: DriveTrain
    val pursuiter: FastPurePursuit
    val localizer: Localizer
    var gyro: Gyro

    // initialize robot
    init {
        val rf = Motor("rf", 1120.0, 1.0, 2.95, hardwareMap)
        val rb = Motor("rb", 1120.0, 1.0, 2.95, hardwareMap)
        val lf = Motor("lf", 1120.0, 1.0, 2.95, hardwareMap)
        val lb = Motor("lb", 1120.0, 1.0, 2.95, hardwareMap)

        gyro = Gyro("gyro", hardwareMap)

        driveTrain = DriveTrain(rf, rb, lf, lb)


        localizer = MecanumLocalizerRev(hardwareMap, gyro=gyro)
//        localizer = MecanumLocalizerRev(hardwareMap, gyro=null)
//
        pursuiter = FastPurePursuit(localizer, Pose2d())

        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (module in allHubs) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }
    }

//    fun move(hori: Double, vert: Double) {
//        telemetry.addData("Encoders", "moving")
//        telemetry.addData("Horizontal", hori)
//        telemetry.addData("Vertical", vert)
//        telemetry.update()
//
//        driveTrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
//        driveTrain.setTarget(DriveTrain.Direction(hori, -vert, 0.0).speeds())
//        driveTrain.setMode(DcMotor.RunMode.RUN_TO_POSITION)
//
//        val allDrive = DriveTrain.Square(0.8, 0.8, 0.8, 0.8)
//
//        driveTrain.start(allDrive)
//
//        val orientation = gyro.measure()
//
//        // do gyro adjustment         |
//        //                            v
//        while (driveTrain.isBusy && !Thread.interrupted()) {
//            localizer.update()
//
//            val turnSquare = if (abs(headingError(orientation)) > 0.02) {
//                DriveTrain.Vector(0.0, 0.0, turnCorrection(orientation)).speeds()
//            } else {
//                DriveTrain.Square(0.0, 0.0, 0.0, 0.0)
//            }
//
//            driveTrain.start(DriveTrain.addSquares(allDrive, turnSquare))
//        }
//        //turn (-headingError(angle)) * gyroAdjust)
//        driveTrain.start(DriveTrain.Square(0.0, 0.0, 0.0, 0.0))
//        driveTrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
//    }
//
//    fun turnTo(degrees: Double) {
//        while (abs(headingError(degrees/360)) > 0.02 && !Thread.interrupted()) {
//            localizer.update()
//
//            telemetry.addData("Gyro Sensor", "turning")
//            telemetry.addData("Angle", gyro.measure())
//            telemetry.update()
//
//            val turn = turnCorrection(degrees)
//            driveTrain.start(DriveTrain.Vector(0.0, 0.0, turn).speeds())
//        }
//
//        driveTrain.start(DriveTrain.Vector(0.0, 0.0, 0.0).speeds())
//    }
//
//    fun turnCorrection(orientation: Double): Double {
//        val rawError = headingError(orientation)
//
//        return rawError * ((1- turnStatic)/0.5) + (turnStatic * sign(rawError))
//    }
//    fun headingError(orientation: Double) : Double{
//        var rawError = orientation - gyro.measure()
//        if (rawError < -0.5) {
//            rawError += 1.0
//        }
//        if (rawError > 0.5) {
//            rawError -= 1.0
//        }
//        return rawError
//    }
//
//
//    companion object {
//        const val turnStatic = 0.2
//    }

    //secondary constructor
//    constructor(hardwareMap: HardwareMap, telemetry: Telemetry) : this(hardwareMap, telemetry, null)
//    {} add code here as constructor body if code for this specific case is needed
}