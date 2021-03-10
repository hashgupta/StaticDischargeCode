package org.firstinspires.ftc.teamcode.robotConfigs

import com.qualcomm.hardware.rev.RevTouchSensor
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Controllers.Arm
import org.firstinspires.ftc.teamcode.Controllers.Shooter
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.hardware.general.ServoM
import org.firstinspires.ftc.teamcode.localizers.TwoWheelRevLocalizer
import org.firstinspires.ftc.teamcode.purePursuit.FastPurePursuit

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
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        roller = Motor("under roller", 560.0, 0.5, hardwareMap)


        val wobble = Motor("wobble", 1120.0,0.75, hardwareMap)
        wobble.device.direction = DcMotorSimple.Direction.REVERSE

        val grabber = ServoM("grabber", hardwareMap)
        grabber.start(1.0)
        arm = Arm(arm_motor = wobble, startAngle = Math.toRadians(130.0), grabber = grabber)


//        val digitalTouch = hardwareMap.digitalChannel.get("touch")
//        digitalTouch.setMode(DigitalChannel.Mode.INPUT);




        flicker = ServoM("flicker", hardwareMap)
        flicker.start(0.9)
        val flywheel = Motor("flywheel", 28.0, 1.0, 4.0, hardwareMap)
        shooter = Shooter(flywheel, Math.toRadians(30.0), 8.0, telemetry, flicker)

        setDriveTrain(rf, rb, lf, lb)


        localizer = TwoWheelRevLocalizer(hardwareMap, "lb", "lf", gyro)

        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        driveTrain.setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE)

        pursuiter = FastPurePursuit(localizer)
    }
}