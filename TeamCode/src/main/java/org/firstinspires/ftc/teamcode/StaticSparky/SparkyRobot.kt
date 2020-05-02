package org.firstinspires.ftc.teamcode.StaticSparky

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.purePursuit.PurePursuitDrive
import org.firstinspires.ftc.teamcode.purePursuit.ThreeWheelRevLocalizer


class SparkyRobot(val hardwareMap: HardwareMap, val telemetry: Telemetry, val opMode: LinearOpMode?) {
    // OpMode members
    var driveTrain: DriveTrain
    var pursuiter: PurePursuitDrive
    //    var lift: Motor
    //    var dropStone: ServoM
    //    var pinch: ServoM
    //    var hook1: ServoM
    //    var hook2: ServoM
    //    var gyro: Gyro
    //    var stone: Color
    //    var distance: Distance

    // initialize robot
    init {
        val rf = Motor("rf", 1120.0, 1.0, 2.95, hardwareMap)
        val rb = Motor("rb", 1120.0, 1.0, 2.95, hardwareMap)
        val lf = Motor("lf", 1120.0, 1.0, 2.95, hardwareMap)
        val lb = Motor("lb", 1120.0, 1.0, 2.95, hardwareMap)

        driveTrain = DriveTrain(rf, rb, lf, lb)

        pursuiter = PurePursuitDrive(ThreeWheelRevLocalizer(hardwareMap))

        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (module in allHubs) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }
    }

    //secondary constructor
    constructor(hardwareMap: HardwareMap, telemetry: Telemetry) : this(hardwareMap, telemetry, null)
//    {} add code here as constructor body if code for this specific case is needed
}