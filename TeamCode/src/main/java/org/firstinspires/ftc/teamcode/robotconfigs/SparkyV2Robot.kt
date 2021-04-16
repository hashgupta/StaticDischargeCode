package org.firstinspires.ftc.teamcode.robotconfigs


import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.controllers.Arm
import org.firstinspires.ftc.teamcode.controllers.Intake
import org.firstinspires.ftc.teamcode.controllers.Shooter
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.hardware.general.ServoNormal
import org.firstinspires.ftc.teamcode.localizers.TwoWheelRevLocalizer
import org.firstinspires.ftc.teamcode.purepursuit.PurePursuit

class SparkyV2Robot(hardwareMap: HardwareMap, telemetry: Telemetry, opModeActive: () -> Boolean) : RobotBase(hardwareMap, telemetry, opModeActive) {
    // OpMode members
    val pursuiter: PurePursuit
    val intake: Intake
    val arm: Arm
    val flicker: ServoNormal
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

        val mainIntake = Motor("intake", 288.0, 1.0, hardwareMap)
        val roller = Motor("under roller", 560.0, 0.5, hardwareMap)

        intake = Intake(arrayOf(mainIntake, roller), arrayOf(-0.8, 0.9))


        val wobble = Motor("wobble", 1120.0, 0.75, hardwareMap)
        wobble.device.direction = DcMotorSimple.Direction.REVERSE

        val grabber = ServoNormal("grabber", hardwareMap)
        grabber.start(0.75)
        arm = Arm(arm_motor = wobble, startAngle = Math.toRadians(130.0), grabber = grabber)


        flicker = ServoNormal("flicker", hardwareMap)
        flicker.start(0.9)

        val flywheel = Motor("flywheel", 28.0, 1.0, 4.0, hardwareMap)
        shooter = Shooter(flywheel, Math.toRadians(30.0), 8.0, telemetry, flicker)

        setDriveTrain(rf, rb, lf, lb)


        localizer = TwoWheelRevLocalizer(hardwareMap, "lb", "lf", gyro)

        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        driveTrain.setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE)

        pursuiter = PurePursuit(localizer)
    }
}