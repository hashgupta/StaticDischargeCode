package org.firstinspires.ftc.teamcode.staticSparky


import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Positions
import org.firstinspires.ftc.teamcode.pipelines.RingPipeline
import org.firstinspires.ftc.teamcode.robotConfigs.SparkyV2Robot

@Autonomous(name = "SparkyAutoRedRight", group = "StaticDischarge")
class SparkyAutoRedRight : GenericOpModeBase() {

    lateinit var robot: SparkyV2Robot

    override fun runOpMode() {
        // UNCOMMENT THIS IF SOUNDS ARE NEEDED
        robot = SparkyV2Robot(hardwareMap, telemetry) { opModeIsActive() && !isStopRequested() }
        robot.pose = Positions.startRightRed


        var analysis = RingPipeline.RingPosition.NONE
        initCV(Side.Left)
        startCV()

        while (opModeIsActive()) {
            analysis = (pipeline as RingPipeline).position()
            telemetry.addData("analysis", analysis)
            telemetry.update()
        }

        waitForStart()
        if (isStopRequested()) return
        stopCV()

        robot.move(0.5 * TILE_LENGTH, 2 * TILE_LENGTH)

        if (analysis == RingPipeline.RingPosition.ONE) {
            robot.toGoal(Pose2d(Positions.bZoneRed, 0.0))
        } else if (analysis == RingPipeline.RingPosition.FOUR) {
            robot.toGoal(Pose2d(Positions.cZoneRed, 0.0))
        } else {
            robot.toGoal(Pose2d(Positions.aZoneRed, 0.0))
        }
        robot.toGoal(Pose2d(0.5 * TILE_LENGTH, -1.5 * TILE_LENGTH, 0.0))

        // first build a sequential list of commands
        // then, program out your instructions one by one
    }
}