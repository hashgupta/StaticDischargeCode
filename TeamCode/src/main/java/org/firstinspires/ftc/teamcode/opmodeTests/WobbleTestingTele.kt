package org.firstinspires.ftc.teamcode.StaticSparky

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robotConfigs.SparkyV2Robot

@TeleOp(name = "Wobble Testing TeleOP", group = "Static Discharge")
class WobbleTestingTele : LinearOpMode() {
    lateinit var robot: SparkyV2Robot

    override fun runOpMode() {
        robot = SparkyV2Robot(hardwareMap, telemetry) { false }


        waitForStart()
        robot.arm.dropAuto()

    }

}