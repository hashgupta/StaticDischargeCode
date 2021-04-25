package org.firstinspires.ftc.teamcode.testopmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.controllers.MecanumDriveTrain
import org.firstinspires.ftc.teamcode.cv.FindRingAutoPipeline
import org.firstinspires.ftc.teamcode.hardware.general.RevPotentiometer
import org.firstinspires.ftc.teamcode.matchopmodes.GenericOpModeBase
import org.firstinspires.ftc.teamcode.robotconfigs.SparkyV2Robot

//@Disabled
@Autonomous(group = "Opmodes for testing", name = "CV test")

class TestPotentiometer : GenericOpModeBase() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {


        val potentio = RevPotentiometer("potentiometer", hardwareMap)

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("measured angle (deg)", potentio.measure())
            telemetry.update()
        }
    }
}