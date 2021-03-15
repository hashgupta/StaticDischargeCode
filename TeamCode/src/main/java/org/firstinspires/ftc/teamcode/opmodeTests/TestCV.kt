package org.firstinspires.ftc.teamcode.opmodeTests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.pipelines.RingPipeline
import org.firstinspires.ftc.teamcode.staticSparky.GenericOpModeBase

@Autonomous(name = "CVAUTOTEST")
class TestCV : GenericOpModeBase() {

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