package org.firstinspires.ftc.teamcode.robotConfigs

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.localizers.TwoWheelRevLocalizer
import org.firstinspires.ftc.teamcode.purePursuit.FastPurePursuit
import org.firstinspires.ftc.teamcode.robotConfigs.RobotBase

class SparkyV2Robot(hardwareMap: HardwareMap, telemetry: Telemetry, opModeActive: () -> Boolean) : RobotBase(hardwareMap, telemetry, opModeActive) {
    // OpMode members
    val pursuiter: FastPurePursuit
//    val intake: Motor

    // initialize robot
    init {
        val rf = Motor("rf", 1120.0, 1.0, 2.95, hardwareMap)
        val rb = Motor("rb", 1120.0, 1.0, 2.95, hardwareMap)
        val lf = Motor("lf", 1120.0, 1.0, 2.95, hardwareMap)
        val lb = Motor("lb", 1120.0, 1.0, 2.95, hardwareMap)

        rf.device.direction = DcMotorSimple.Direction.REVERSE
        rb.device.direction = DcMotorSimple.Direction.REVERSE
        lf.device.direction = DcMotorSimple.Direction.REVERSE
        lb.device.direction = DcMotorSimple.Direction.REVERSE

//        intake = Motor("intake", 1120.0, 1.0, 2.95, hardwareMap)

        setDriveTrain(rf, rb, lf, lb)

        localizer = TwoWheelRevLocalizer(hardwareMap, "lf", "rb", gyro)

        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        pursuiter = FastPurePursuit(localizer, Pose2d())
    }
}