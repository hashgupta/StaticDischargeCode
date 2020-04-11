package org.firstinspires.ftc.teamcode.StaticSparky

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "CVAUTOTEST")
class SparkAutoCV : SparkAutoBase() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initCV()
        waitForStart()
        startCV()
        while (opModeIsActive()) {
            telemetry.addData("average", pipeline.average())
            telemetry.update()
        }
        stopCV()
    }
}