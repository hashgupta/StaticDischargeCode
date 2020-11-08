package org.firstinspires.ftc.teamcode.staticSparky

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.pipelines.RingPipeline

@Autonomous(name = "SparkyAutoRedLeft", group = "StaticDischarge")
class SparkyRedAutoLeft : SparkAutoBase() {
    override fun runOpMode() {
        // UNCOMMENT THIS IF SOUNDS ARE NEEDED
        val robot = TestRobot(hardwareMap, telemetry)
        var analysis = RingPipeline.RingPosition.NONE
        initCV(Side.Right)
        startCV()
        while (opModeIsActive()) {
            analysis = pipeline.position()
            telemetry.addData("analysis",analysis)
            telemetry.update()
        }
        waitForStart()
        stopCV()

        robot.move(-0.5 * TILE_LENGTH, 2 * TILE_LENGTH)

        if (analysis == RingPipeline.RingPosition.ONE) {
            robot.toGoal(Pose2d(4 * TILE_LENGTH, 0.5* TILE_LENGTH, 0.0))
            robot.toGoal(Pose2d(3 * TILE_LENGTH, 0.5* TILE_LENGTH, 0.0))
        } else if (analysis == RingPipeline.RingPosition.FOUR) {
            robot.toGoal(Pose2d(5 * TILE_LENGTH, 1.5* TILE_LENGTH, 0.0))
            robot.toGoal(Pose2d(3 * TILE_LENGTH, 0.5* TILE_LENGTH, 0.0))
        } else {
            robot.toGoal(Pose2d(3* TILE_LENGTH, 1.5* TILE_LENGTH, 0.0))
        }
//        robot.turnTo(180.0)
//        robot.move(0.0, TILE_LENGTH)

        // first build a sequential list of commands
        // then, program out your instructions one by one
    }
}