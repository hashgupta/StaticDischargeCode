package org.firstinspires.ftc.teamcode.staticSparky

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.ReadWriteFile
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.pipelines.RingPipeline
import java.io.File
import kotlin.math.PI


@Autonomous(name = "SparkyAutoRedLeft", group = "StaticDischarge")
class SparkyRedAutoLeft : SparkAutoBase() {
    override fun runOpMode() {
        // UNCOMMENT THIS IF SOUNDS ARE NEEDED
        robot = SparkyRobot(hardwareMap, telemetry) { opModeIsActive() && !isStopRequested() }
        robot.pose = Positions.startLeftRed
        robot.pursuiter.setStartPoint(Positions.startLeftRed)

        initCV(Side.Right)
        startCV()

        waitForStart()
        val analysis: RingPipeline.RingPosition = pipeline.position()
        telemetry.addData("analysis",analysis)
        telemetry.update()
        stopCV()

        robot.pursuiter.addAction { robot.arm.toAngle(0.0) }

        robot.pursuiter.addRelativePoint(-0.5 * TILE_LENGTH, 2 * TILE_LENGTH, 0.0)

        val goalZone: Pose2d

        if (analysis == RingPipeline.RingPosition.ONE) {
            goalZone = Pose2d(Positions.bZoneRed, 0.0)
        } else if (analysis == RingPipeline.RingPosition.FOUR) {
            goalZone = Pose2d(Positions.cZoneRed, 0.0)
        } else {
            goalZone = Pose2d(Positions.aZoneRed, 0.0)
        }
        robot.pursuiter.addPoint(goalZone + Pose2d(-9.0, -9.0))
        robot.pursuiter.addAction { robot.arm.dropAuto() }
        robot.pursuiter.addPoint(Positions.startRightRed + Pose2d(TILE_LENGTH*0.75, -Constants.trackwidth * 0.5, PI ))
        robot.pursuiter.addAction { robot.arm.grabAuto() }
        robot.pursuiter.addPoint(goalZone + Pose2d(-9.0, -4.0))
        robot.pursuiter.addAction { robot.arm.dropAuto() }

        robot.pursuiter.addPoint(0.5 * TILE_LENGTH, -1.5* TILE_LENGTH, 0.0)
        robot.pursuiter.FollowSync(robot.driveTrain, telemetry = telemetry)


        robot.finishAuto()

    }
}