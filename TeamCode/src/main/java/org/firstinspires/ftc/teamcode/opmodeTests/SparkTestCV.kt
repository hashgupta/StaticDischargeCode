package org.firstinspires.ftc.teamcode.opmodeTests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.pipelines.RingPipeline
import org.firstinspires.ftc.teamcode.staticSparky.SparkOpModeBase

@Autonomous(name = "CVAUTOTEST")
class SparkTestCV : SparkOpModeBase() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initCV(Side.Left)
        startCV()
        waitForStart()
        while (opModeIsActive()) {
            telemetry.addData("analysis", (pipeline as RingPipeline).position())
            telemetry.update()
        }
        stopCV()
    }
}