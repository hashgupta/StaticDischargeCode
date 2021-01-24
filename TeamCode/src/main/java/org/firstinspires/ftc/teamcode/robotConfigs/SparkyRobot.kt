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
import org.firstinspires.ftc.teamcode.purePursuit.FastPurePursuit
import org.firstinspires.ftc.teamcode.robotConfigs.RobotBase


class SparkyRobot(hardwareMap: HardwareMap, telemetry: Telemetry, opModeActive: () -> Boolean) : RobotBase(hardwareMap, telemetry, opModeActive) {
    // OpMode members

    val pursuiter: FastPurePursuit
    var intakeBottom: Motor
    var intakeTop: Motor
    var flywheel: Motor
    val shooter: Shooter
    val arm: Arm

    // initialize robot
    init {
        val rf = Motor("rf", 1120.0, 1.0, 2.95, hardwareMap)
        val rb = Motor("rb", 1120.0, 1.0, 2.95, hardwareMap)
        val lf = Motor("lf", 1120.0, 1.0, 2.95, hardwareMap)
        val lb = Motor("lb", 1120.0, 1.0, 2.95, hardwareMap)
        setDriveTrain(rf, rb, lf, lb)
        val flicker = ServoM("flicker", hardwareMap)
//        val flicker = ServoCRWrapper("flicker", hardwareMap)
        flicker.start(0.2)

        intakeBottom = Motor("intakeBottom", 1120.0, 1.0, 1.0, hardwareMap)
        intakeTop = Motor("intakeTop", 1120.0, 1.0, 1.0, hardwareMap)
        intakeTop.device.direction = DcMotorSimple.Direction.REVERSE

        flywheel = Motor("flywheel", 28.0, 1.0, 4.0, hardwareMap)
        flywheel.device.direction = DcMotorSimple.Direction.REVERSE
        shooter = Shooter(flywheel, Math.toRadians(45.0), 8.0, telemetry,flicker)

        val wobble_arm = Motor("wobble arm", 288.0, 1.0, hardwareMap)
        wobble_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        wobble_arm.setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        val wobble_grabber = ServoM("wobble grabber", hardwareMap)
        wobble_grabber.start(0.00)
        arm = Arm(Math.toRadians(180.0), wobble_arm, wobble_grabber)



//        localizer = TwoWheelRevLocalizer(hardwareMap, "front", "side")
//
        pursuiter = FastPurePursuit(localizer, Pose2d())
    }
}
