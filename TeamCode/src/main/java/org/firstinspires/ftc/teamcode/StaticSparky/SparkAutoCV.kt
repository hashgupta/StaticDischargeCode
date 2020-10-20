package org.firstinspires.ftc.teamcode.StaticSparky

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.pipelines.RingPipeline

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