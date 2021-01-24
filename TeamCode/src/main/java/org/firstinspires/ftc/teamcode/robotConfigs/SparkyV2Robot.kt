package org.firstinspires.ftc.teamcode.robotConfigs

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Controllers.Arm
import org.firstinspires.ftc.teamcode.Controllers.Shooter
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.hardware.general.ServoM
import org.firstinspires.ftc.teamcode.localizers.TwoWheelRevLocalizer
import org.firstinspires.ftc.teamcode.purePursuit.FastPurePursuit
import org.firstinspires.ftc.teamcode.robotConfigs.RobotBase

class SparkyV2Robot(hardwareMap: HardwareMap, telemetry: Telemetry, opModeActive: () -> Boolean) : RobotBase(hardwareMap, telemetry, opModeActive) {
    // OpMode members
    val pursuiter: FastPurePursuit
    val intake: Motor
    val roller: Motor
    val arm : Arm
    val flicker : ServoM
    val shooter: Shooter


    // initialize robot
    init {
        val rf = Motor("rf", 529.0, 1.0, 2.95, hardwareMap)
        val rb = Motor("rb", 529.0, 1.0, 2.95, hardwareMap)
        val lf = Motor("lf", 529.0, 1.0, 2.95, hardwareMap)
        val lb = Motor("lb", 529.0, 1.0, 2.95, hardwareMap)

        rf.device.direction = DcMotorSimple.Direction.REVERSE
        rb.device.direction = DcMotorSimple.Direction.REVERSE
        lf.device.direction = DcMotorSimple.Direction.REVERSE
        lb.device.direction = DcMotorSimple.Direction.REVERSE

        intake = Motor("intake", 288.0, 1.0, hardwareMap)
        roller = Motor("under roller", 560.0, 0.5, hardwareMap)
        val wobble = Motor("wobble", 1120.0,2.0, hardwareMap)
        wobble.device.direction = DcMotorSimple.Direction.REVERSE
        arm = Arm(arm_motor = wobble, startAngle = Math.toRadians(180.0), grabber = null)




        flicker = ServoM("flicker", hardwareMap)
        flicker.start(0.9)
        val flywheel = Motor("flywheel", 28.0, 1.0, 4.0, hardwareMap)
        shooter = Shooter(flywheel, Math.toRadians(30.0), 8.0, telemetry, flicker)

        setDriveTrain(rf, rb, lf, lb)

        localizer = TwoWheelRevLocalizer(hardwareMap, "lb", "lf", gyro)

        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        driveTrain.setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE)

        pursuiter = FastPurePursuit(localizer, Pose2d())
    }
}