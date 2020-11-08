package org.firstinspires.ftc.teamcode.staticSparky

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "CVAUTOTEST")
class SparkAutoCV : SparkAutoBase() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initCVNoWebcam(Side.Left)
        startCV()
        waitForStart()
        while (opModeIsActive()) {
            telemetry.addData("analysis", pipeline.position())
            telemetry.update()
        }
        stopCV()
    }
}