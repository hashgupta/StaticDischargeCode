package org.firstinspires.ftc.teamcode.opmodeTests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.cvPipelines.RingPipeline
import org.firstinspires.ftc.teamcode.matchOpmodes.GenericOpModeBase

@Autonomous(group = "Opmodes for testing", name = "CV test")
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