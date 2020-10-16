package org.firstinspires.ftc.teamcode.StaticSparky

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "CVAUTOTEST")
class SparkAutoCV : SparkAutoBase() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initCVNoWebcam(Side.Right)
        startCV()
        waitForStart()
        while (opModeIsActive()) {
            telemetry.addData("average", pipeline.average())
            telemetry.addData("analysis", pipeline.position())
            telemetry.update()
        }
        stopCV()
    }
}