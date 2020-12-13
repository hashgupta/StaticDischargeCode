package org.firstinspires.ftc.teamcode.staticSparky

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.ReadWriteFile
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.pipelines.RingPipeline
import java.io.File
import java.util.*
import kotlin.math.PI


@Autonomous(name = "SparkyAutoRedLeft", group = "StaticDischarge")
class SparkyRedAutoLeft : SparkAutoBase() {
    override fun runOpMode() {
        // UNCOMMENT THIS IF SOUNDS ARE NEEDED
        robot = SparkyRobot(hardwareMap, telemetry) { opModeIsActive() && !isStopRequested() }
        robot.pursuiter.setStartPoint(Positions.startLeftRed)
        initCV(Side.Right)
        startCV()
        var analysis: RingPipeline.RingPosition = RingPipeline.RingPosition.NONE
        while (!isStarted) {
            analysis= pipeline.position()
            telemetry.addData("analysis",analysis)
            telemetry.update()
        }

        //start button has been hit
        // main auto code begins
        stopCV()

        robot.pursuiter.addRelativePoint(-0.5*TILE_LENGTH, 2* TILE_LENGTH, 0.0)
        robot.pursuiter.addPoint(Pose2d(0.0, -TILE_LENGTH*1.5,
                robot.shooter.turningTarget(Vector2d(0.0, -TILE_LENGTH*1.5), Positions.highGoalRed) + PI)
        )
        robot.pursuiter.FollowSync(robot.driveTrain, telemetry = telemetry)
        robot.pursuiter.setStartPoint(robot.localizer.poseEstimate)

        robot.pursuiter.addAction {
            robot.shooter.simpleShootAtTarget(robot.localizer.poseEstimate, Positions.highGoalRed)
            sleep(2000)
            robot.shooter.shoot()
            robot.shooter.shoot()
            robot.shooter.shoot()
        }


//        robot.pursuiter.addTurnAbsolute(robot.shooter.turningTarget(robot.pursuiter.waypoints.last().end.vec(), Positions.powerNearRed) + PI)
//        robot.pursuiter.addAction {  robot.shooter.simpleShootAtTarget(robot.localizer.poseEstimate, Positions.powerNearRed); sleep(1000); robot.shooter.shoot()}
//
//        robot.pursuiter.addTurnAbsolute(robot.shooter.turningTarget(robot.pursuiter.waypoints.last().end.vec(), Positions.powerMidRed) + PI)
//        robot.pursuiter.addAction {  robot.shooter.simpleShootAtTarget(robot.localizer.poseEstimate, Positions.powerMidRed)
//            sleep(1000)
//            robot.shooter.shoot() }
//        robot.pursuiter.addTurnAbsolute(robot.shooter.turningTarget(robot.pursuiter.waypoints.last().end.vec(), Positions.powerFarRed) + PI)
//        robot.pursuiter.addAction { robot.shooter.simpleShootAtTarget(robot.localizer.poseEstimate, Positions.powerFarRed)
//            sleep(1000)
//            robot.shooter.shoot() }

//        val goalZone = Pose2d(Positions.bZoneRed, 0.0)

//        val goalZone: Pose2d
//        if (analysis == RingPipeline.RingPosition.NONE) {
//            goalZone = Pose2d(Positions.aZoneRed, 0.0)
//        } else if (analysis == RingPipeline.RingPosition.ONE) {
//            goalZone = Pose2d(Positions.bZoneRed, 0.0)
//        } else {
//            goalZone = Pose2d(Positions.cZoneRed, 0.0)
//        }
////        robot.pursuiter.setStartPoint(robot.localizer.poseEstimate)

//        robot.pursuiter.addPoint(goalZone + Pose2d(-9.0, 0.0))
//        robot.pursuiter.addAction { robot.arm.dropAuto() }

//        robot.pursuiter.addPoint(Positions.startRightRed + Pose2d(SparkAutoBase.TILE_LENGTH *0.5 + 9.0, Constants.trackwidth * 0.5, PI ))
//        robot.pursuiter.addAction { robot.arm.grabAuto() }
//
//        robot.pursuiter.addPoint(goalZone + Pose2d(-9.0, 0.0))
//        robot.pursuiter.addAction { robot.arm.dropAuto()}

        robot.pursuiter.addPoint(0.5 * TILE_LENGTH, -2* TILE_LENGTH, PI)
        //the entire auto program above is dynamically building the path and actions
        // the line below does all the activity
        robot.pursuiter.FollowSync(robot.driveTrain, telemetry = telemetry)

        //record position to the file for teleop readings
        robot.finishAuto()

    }
}