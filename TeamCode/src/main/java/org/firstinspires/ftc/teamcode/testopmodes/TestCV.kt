package org.firstinspires.ftc.teamcode.testopmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.controllers.MecanumDriveTrain
import org.firstinspires.ftc.teamcode.cv.FindRingAutoPipeline
import org.firstinspires.ftc.teamcode.matchopmodes.GenericOpModeBase
import org.firstinspires.ftc.teamcode.robotconfigs.SparkyV2Robot

//@Disabled
@Autonomous(group = "Opmodes for testing", name = "CV test")

class TestCV : GenericOpModeBase() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        pipeline = FindRingAutoPipeline()
        initCV()
        startCV()

        val robot = SparkyV2Robot(hardwareMap, telemetry) {true}
        waitForStart()
        robot.intake.on(forward = true)

        while (opModeIsActive()) {
            telemetry.addLine("running opencv pipeline")
//            telemetry.addData("rings", (pipeline as FindRingAutoPipeline).rings())
            telemetry.update()
            val rings = (pipeline as FindRingAutoPipeline).rings()
            robot.intake.run()
            if (rings.isNotEmpty()) {
                if (rings[0].distance < 15) {
                    robot.driveTrain.start(MecanumDriveTrain.Vector(hori = 0.0, vert = rings[0].distance / 15.0, turn = 0.0).speeds())
                    sleep(1000)
                }
                robot.driveTrain.start(MecanumDriveTrain.Vector(hori = 0.0, vert = rings[0].distance / 15.0, turn = rings[0].turnControl * 4).speeds())
            } else {
                robot.driveTrain.start(MecanumDriveTrain.Square(0.0, 0.0, 0.0, 0.0))
            }
        }
        stopCV()
    }
}