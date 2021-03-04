package org.firstinspires.ftc.teamcode.robotConfigs

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.firstinspires.ftc.teamcode.PoseStorage
import org.firstinspires.ftc.teamcode.hardware.general.Gyro
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.localizers.MecanumLocalizerRev
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sign

abstract class RobotBase(val hardwareMap: HardwareMap, val telemetry: Telemetry, val opModeActive: () -> Boolean) {
    var driveTrain: DriveTrain
    var localizer: Localizer
    val gyro: Gyro
    var pose: Pose2d = Pose2d(0.0, 0.0, 0.0)


    init {
        val rf = Motor("rf", 1120.0, 1.0, 2.95, hardwareMap)
        val rb = Motor("rb", 1120.0, 1.0, 2.95, hardwareMap)
        val lf = Motor("lf", 1120.0, 1.0, 2.95, hardwareMap)
        val lb = Motor("lb", 1120.0, 1.0, 2.95, hardwareMap)

        driveTrain = DriveTrain(rf, rb, lf, lb)

        gyro = Gyro("gyro", hardwareMap)

        localizer = MecanumLocalizerRev(hardwareMap, gyro = gyro)


        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (module in allHubs) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }
    }

    fun move(hori: Double, vert: Double) {
        telemetry.addData("Encoders", "moving")
        telemetry.addData("Horizontal", hori)
        telemetry.addData("Vertical", vert)


        driveTrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        telemetry.addData("drivetrain", driveTrain.getPosition())
        telemetry.update()
        Thread.sleep(1000)
        driveTrain.setTarget(DriveTrain.Direction(hori, -vert, 0.0).speeds())
        driveTrain.setMode(DcMotor.RunMode.RUN_TO_POSITION)


        val allDrive = DriveTrain.Square(0.8, 0.8, 0.8, 0.8)

        driveTrain.start(allDrive)

//        val orientation = gyro.measureRadians()


        // do gyro adjustment         |
        //                            v
        while (driveTrain.isBusy && opModeActive()) {
            telemetry.addData("Drivetrain", driveTrain.getPosition())
            telemetry.addData("Target", driveTrain.getTarget())
            telemetry.update()
//            localizer.update()

//            val turnSquare = if (abs(headingError(orientation)) > 0.02) {
//                DriveTrain.Vector(0.0, 0.0, turnCorrection(orientation)).speeds()
//            } else {
//                DriveTrain.Square(0.0, 0.0, 0.0, 0.0)
//            }
//            val turnSquare =  DriveTrain.Square(0.0, 0.0, 0.0, 0.0)
//            driveTrain.start(DriveTrain.addSquares(allDrive, turnSquare))
        }
        val robotPoseDelta = Pose2d(vert, -hori, 0.0)
        pose = Kinematics.relativeOdometryUpdate(pose, robotPoseDelta)
        driveTrain.start(DriveTrain.Square(0.0, 0.0, 0.0, 0.0))
        driveTrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
    }

    fun turnTo(radians: Double) {
        while (abs(headingError(radians / (2* PI))) > 0.02 && opModeActive()) {
//            localizer.update()

            val turn = turnCorrection(radians/(2* PI))
            telemetry.addData("Gyro Sensor Off", headingError(radians / (2 * PI)))
            telemetry.addData("Angle", Math.toDegrees(gyro.measure()))
            telemetry.update()

            driveTrain.start(DriveTrain.Vector(0.0, 0.0, -turn).speeds())
        }
        driveTrain.start(DriveTrain.Vector(0.0, 0.0, 0.0).speeds())
        pose = Pose2d(pose.vec(), gyro.measure())
    }

    fun toGoal(goalPose: Pose2d) {
        val error = Kinematics.calculatePoseError(goalPose, pose)
        this.move(-error.y, error.x)
        this.turnTo(goalPose.heading)
    }

    fun turnCorrection(orientation: Double): Double {
        val rawError = headingError(orientation)
        telemetry.addData("Raw Error correction", rawError)

        return rawError * ((1 - turnStatic) / 0.5) + (turnStatic * sign(rawError))
    }

    fun headingError(orientation: Double): Double {
        var rawError = orientation - gyro.measure()
        if (rawError < -0.5) {
            rawError += 1.0
        }
        if (rawError > 0.5) {
            rawError -= 1.0
        }
        return rawError
    }

    companion object {
        const val turnStatic = 0.25
    }

    fun setDriveTrain(rf: Motor, rb: Motor, lf: Motor, lb: Motor) {
        driveTrain = DriveTrain(rf, rb, lf, lb)
    }

    fun savePose() {
        PoseStorage.pose = this.localizer.poseEstimate
//        val filename = "position.json"
//        val file: File = AppUtil.getInstance().getSettingsFile(filename)
//
//        val data = this.localizer.poseEstimate.x.toString() + " " + this.localizer.poseEstimate.y.toString() + " " + this.localizer.poseEstimate.heading.toString()
//        ReadWriteFile.writeFile(file, data)
    }

    fun loadPose() {
        this.localizer.poseEstimate = PoseStorage.pose
    }
}