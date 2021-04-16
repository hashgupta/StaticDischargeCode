package org.firstinspires.ftc.teamcode.testopmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.cv.RingPipeline
import org.firstinspires.ftc.teamcode.matchopmodes.GenericOpModeBase
@Disabled
@Autonomous(group = "Opmodes for testing", name = "CV test")

class TestCV : GenericOpModeBase() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        setUpPipeline(true)
        initCV()
        startCV()
        waitForStart()
        while (opModeIsActive()) {
            telemetry.addData("analysis", (pipeline as RingPipeline).position())
            telemetry.update()
        }
        stopCV()
    }
}