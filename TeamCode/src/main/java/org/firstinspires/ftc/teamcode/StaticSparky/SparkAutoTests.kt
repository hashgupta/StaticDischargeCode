package org.firstinspires.ftc.teamcode.StaticSparky

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "FujiTests", group = "PatentPending")
class SparkAutoTests : SparkAutoBase() {

    override fun runOpMode() {
        // UNCOMMENT THIS IF SOUNDS ARE NEEDED
        robot = SparkyRobot(hardwareMap, telemetry)
        waitForStart()


    }
}