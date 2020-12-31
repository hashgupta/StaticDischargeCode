package org.firstinspires.ftc.teamcode.staticSparky

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "CVAUTOTEST")
class SparkTestCV : SparkOpModeBase() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initCV(Side.Left)
        startCV()
        waitForStart()
        while (opModeIsActive()) {
            telemetry.addData("analysis", pipeline.position())
            telemetry.update()
        }
        stopCV()
    }
}