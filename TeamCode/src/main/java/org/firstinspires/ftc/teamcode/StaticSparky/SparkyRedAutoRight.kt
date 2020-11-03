package org.firstinspires.ftc.teamcode.StaticSparky


import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.apache.commons.math3.geometry.euclidean.twod.PolygonsSet
import org.firstinspires.ftc.teamcode.pipelines.RingPipeline

@Autonomous(name = "SparkyAutoRedRight", group = "StaticDischarge")
class SparkyRedAutoRight : SparkAutoBase() {
    override fun runOpMode() {
        // UNCOMMENT THIS IF SOUNDS ARE NEEDED
        val robot = TestRobot(hardwareMap, telemetry)
        var analysis = RingPipeline.RingPosition.NONE
        initCV(Side.Left)
        startCV()
        while (opModeIsActive()) {
            analysis = pipeline.position()
            telemetry.addData("analysis", analysis)
            telemetry.update()
        }
        waitForStart()
        stopCV()

        robot.move(0.5 * TILE_LENGTH, 2 * TILE_LENGTH)

        if (analysis == RingPipeline.RingPosition.ONE) {
            robot.toGoal(Pose2d(4 * TILE_LENGTH, 0.5 * TILE_LENGTH, 0.0))
            robot.toGoal(Pose2d(3 * TILE_LENGTH, 0.5 * TILE_LENGTH, 0.0))
        } else if (analysis == RingPipeline.RingPosition.FOUR) {
            robot.toGoal(Pose2d(5 * TILE_LENGTH, 1.5 * TILE_LENGTH, 0.0))
            robot.toGoal(Pose2d(3 * TILE_LENGTH, 0.5 * TILE_LENGTH, 0.0))
        } else {
            robot.toGoal(Pose2d(3 * TILE_LENGTH, 1.5 * TILE_LENGTH, 0.0))
        }

        // first build a sequential list of commands
        // then, program out your instructions one by one
    }
}