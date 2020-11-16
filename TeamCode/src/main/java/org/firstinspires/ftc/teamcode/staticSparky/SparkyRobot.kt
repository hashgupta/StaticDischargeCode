package org.firstinspires.ftc.teamcode.staticSparky

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Controllers.Shooter
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.hardware.general.ServoM


class SparkyRobot(hardwareMap: HardwareMap, telemetry: Telemetry, opModeActive: () -> Boolean) : RobotBase(hardwareMap, telemetry, opModeActive) {
    // OpMode members

//    val pursuiter: FastPurePursuit
    var intakeBottom: Motor
    var intakeTop: Motor
    var flywheel: Motor
    val shooter: Shooter
    //    var dropStone: ServoM
    //    var pinch: ServoM
    //    var hook1: ServoM
    //    var hook2: ServoM
//    val touch: Touch
    //    var stone: Color
    //    var distance: Distance

    // initialize robot
    init {
        val rf = Motor("rf", 1120.0, 1.0, 2.95, hardwareMap)
        val rb = Motor("rb", 1120.0, 1.0, 2.95, hardwareMap)
        val lf = Motor("lf", 1120.0, 1.0, 2.95, hardwareMap)
        val lb = Motor("lb", 1120.0, 1.0, 2.95, hardwareMap)
        setDriveTrain(rf, rb, lf, lb)
//        val flicker = ServoM("flicker", hardwareMap)
        val flicker = null
        intakeBottom = Motor("intakeBottom", 1120.0, 1.0, 1.0, hardwareMap)
        intakeTop = Motor("intakeTop", 1120.0, 1.0, 1.0, hardwareMap)
        intakeTop.device.direction = DcMotorSimple.Direction.REVERSE
        flywheel = Motor("flywheel", 1120.0, 17.36, 4.0, hardwareMap)
        shooter = Shooter(flywheel, Math.toRadians(45.0), 11.0, flicker)

//        localizer = TwoWheelRevLocalizer(hardwareMap, "front", "side")
//
//        pursuiter = FastPurePursuit(localizer, Pose2d())
    }
}
