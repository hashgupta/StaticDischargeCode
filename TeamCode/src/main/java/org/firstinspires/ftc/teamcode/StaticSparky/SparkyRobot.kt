package org.firstinspires.ftc.teamcode.StaticSparky

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.localizers.TwoWheelRevLocalizer
import org.firstinspires.ftc.teamcode.purePursuit.FastPurePursuit


class SparkyRobot(hardwareMap: HardwareMap, telemetry: Telemetry) : RobotBase(hardwareMap, telemetry) {
    // OpMode members

    val pursuiter: FastPurePursuit
    //    var lift: Motor
    //    var dropStone: ServoM
    //    var pinch: ServoM
    //    var hook1: ServoM
    //    var hook2: ServoM
//    val touch: Touch
    //    var stone: Color
    //    var distance: Distance

    // initialize robot
    init {
        localizer = TwoWheelRevLocalizer(hardwareMap, "front", "side")

        pursuiter = FastPurePursuit(localizer, Pose2d())
    }
}
