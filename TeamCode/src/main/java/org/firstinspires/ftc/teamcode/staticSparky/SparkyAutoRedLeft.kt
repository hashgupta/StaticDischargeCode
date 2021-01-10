package org.firstinspires.ftc.teamcode.staticSparky

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.pipelines.RingPipeline
import kotlin.math.PI


@Autonomous(name = "SparkyAutoRedLeft", group = "StaticDischarge")
class SparkyAutoRedLeft : SparkOpModeBase() {
    override fun runOpMode() {
        // UNCOMMENT THIS IF SOUNDS ARE NEEDED
        robot = SparkyRobot(hardwareMap, telemetry) { opModeIsActive() && !isStopRequested() }
        robot.pursuiter.setStartPoint(Positions.startLeftRed)
        initCV(Side.Right)
        startCV()
        var analysis: RingPipeline.RingPosition = RingPipeline.RingPosition.NONE
        while (!isStarted) {
            analysis= (pipeline as RingPipeline).position()
            telemetry.addData("analysis",analysis)
            telemetry.update()
        }

        //start button has been hit
        // main auto code begins
        stopCV()

        //NOTE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! %$%#@#$%#@@#$@#$%#Q$%^@#^&
        // i changed the start left red to start at Pi radians, so just start the robot flipped
        // then you wont have to turn to shoot

        robot.pursuiter.addRelativePoint(-0.5*TILE_LENGTH, 2* TILE_LENGTH, 0.0)

        robot.pursuiter.addPoint(Pose2d(-0.0* TILE_LENGTH, Positions.highGoalRed.y, robot.shooter.turningTarget(Vector2d(-0.0* TILE_LENGTH, Positions.highGoalRed.y), Positions.highGoalRed)))
        robot.pursuiter.addAction {robot.shooter.simpleShootAtTarget(robot.localizer.poseEstimate,
                    Positions.highGoalRed); sleep(1500);  robot.shooter.shoot()
        sleep(500); robot.shooter.shoot(); sleep(500);robot.shooter.shoot()}



//        robot.pursuiter.addPoint(Pose2d(-0.5* TILE_LENGTH, Positions.powerFarRed.y, PI))
//        robot.pursuiter.addAction {robot.shooter.simpleShootAtTarget(robot.localizer.poseEstimate,
//                    Positions.powerFarRed); sleep(1500);  robot.shooter.shoot()}
//
//        robot.pursuiter.addPoint(Pose2d(-0.5* TILE_LENGTH, Positions.powerMidRed.y, PI))
//        robot.pursuiter.addAction {robot.shooter.simpleShootAtTarget(robot.localizer.poseEstimate,
//                Positions.powerMidRed); sleep(500);  robot.shooter.shoot()}
//
//        robot.pursuiter.addPoint(Pose2d(-0.5* TILE_LENGTH, Positions.powerNearRed.y, PI))
//        robot.pursuiter.addAction {robot.shooter.simpleShootAtTarget(robot.localizer.poseEstimate,
//                Positions.powerNearRed); sleep(500);  robot.shooter.shoot()}



//        val goalZone = when (analysis) {
//            RingPipeline.RingPosition.NONE -> {
//                Pose2d(Positions.aZoneRed, 0.0)
//            }
//            RingPipeline.RingPosition.ONE -> {
//                Pose2d(Positions.bZoneRed, 0.0)
//            }
//            else -> {
//                Pose2d(Positions.cZoneRed, 0.0)
//            }
//        }
////
//        robot.pursuiter.addPoint(goalZone + Pose2d(-24.0, 12.0, -PI/4))
//        robot.pursuiter.addAction { robot.arm.toAngle(Math.toRadians(0.0))
//            while ( robot.arm.arm_motor.isBusy) {
//
//            }
//            robot.arm.run(0.0)
//            sleep(500)
//            robot.arm.dropAuto() }

//        robot.pursuiter.addPoint(-1* TILE_LENGTH,-2.25* TILE_LENGTH, PI)
//        robot.pursuiter.addAction {  }
////
//        robot.pursuiter.addPoint(Positions.startRightRed + Pose2d(SparkAutoBase.TILE_LENGTH *0.5 + 12.0, Constants.trackwidth * 0.5, PI ))
//        robot.pursuiter.addAction { robot.arm.grabAuto() }
////
////        robot.pursuiter.addPoint(goalZone + Pose2d(-9.0, 0.0))
////        robot.pursuiter.addAction { robot.arm.dropAuto()}

        robot.pursuiter.addPoint(0.5 * TILE_LENGTH, -1* TILE_LENGTH, PI)
        robot.pursuiter.addAction {robot.shooter.stopWheel()  }


        //the entire auto program above is dynamically building the path and actions
        // the line below does all the activity
        robot.pursuiter.FollowSync(robot.driveTrain, telemetry = telemetry)

        //record position to the file for teleop readings
        robot.finishAuto()

    }
}