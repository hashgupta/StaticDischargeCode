package org.firstinspires.ftc.teamcode.staticSparky

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Positions
import org.firstinspires.ftc.teamcode.pipelines.RingPipeline
import org.firstinspires.ftc.teamcode.robotConfigs.SparkyRobot
import org.firstinspires.ftc.teamcode.robotConfigs.SparkyV2Robot
import kotlin.math.PI


@Autonomous(name = "SparkyAutoRedLeft", group = "StaticDischarge")
class SparkyAutoRedLeft : SparkOpModeBase() {

    lateinit var robot: SparkyV2Robot

    override fun runOpMode() {
        // UNCOMMENT THIS IF SOUNDS ARE NEEDED

        /*
        **************
        * I changed the roadrunner speed in drivetrain to 0.5 to test out odometry without full speed
        * change back if full speed it wanted
        * also, use the ftc dashboard for path and robot tracking, not the telemetry
        * 192.168.43.1/dash on the same wifi
        **************
        */



        robot = SparkyV2Robot(hardwareMap, telemetry) { opModeIsActive() && !isStopRequested() }
        robot.pursuiter.setStartPoint(Positions.startLeftRed)
        waitForStart()
//        initCV(Side.Right)
//        startCV()
//        var analysis: RingPipeline.RingPosition = RingPipeline.RingPosition.NONE
//        while (!isStarted) {
//            analysis= (pipeline as RingPipeline).position()
//            telemetry.addData("analysis",analysis)
//            telemetry.update()
//        }

        //start button has been hit
        // main auto code begins
//        stopCV()

        //NOTE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // i changed the start left red to start at 0.0 radians, so just start the facing straight with webcam

        robot.pursuiter.addPoint(-TILE_LENGTH, -0.5* TILE_LENGTH, PI)


        robot.pursuiter.addPoint(Pose2d(-0.5* TILE_LENGTH, Positions.highGoalRed.y, robot.shooter.turningTarget(Vector2d(-0.0* TILE_LENGTH, Positions.highGoalRed.y), Positions.highGoalRed)))
        robot.pursuiter.addAction {robot.shooter.simpleShootAtTarget(robot.localizer.poseEstimate,
                    Positions.highGoalRed); sleep(3500);  robot.shooter.shoot()
        sleep(1250); robot.shooter.shoot(); sleep(1250);robot.shooter.shoot()}

        robot.pursuiter.addAction {robot.shooter.stopWheel()  }



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
//
//        robot.pursuiter.addPoint(goalZone + Pose2d(-24.0, 12.0, -PI/4))
//        robot.pursuiter.addAction{ robot.arm.toAngle(Math.toRadians(0.0))
//            robot.arm.dropAuto()
//        }

//        robot.pursuiter.addPoint(-1* TILE_LENGTH,-2.25* TILE_LENGTH, PI)
//        robot.pursuiter.addAction {  }
////
//        robot.pursuiter.addPoint(Positions.startRightRed + Pose2d(SparkAutoBase.TILE_LENGTH *0.5 + 12.0, Constants.trackwidth * 0.5, PI ))
//        robot.pursuiter.addAction { robot.arm.grabAuto() }
////
////        robot.pursuiter.addPoint(goalZone + Pose2d(-9.0, 0.0))
////        robot.pursuiter.addAction { robot.arm.dropAuto()}

        robot.pursuiter.addPoint(0.5 * TILE_LENGTH, -1* TILE_LENGTH, PI)



        // the entire auto program above is dynamically building the path and actions
        // the line below does all the activity
        robot.pursuiter.FollowSync(robot.driveTrain, telemetry = telemetry)

        //record position for teleop
        robot.savePose()

    }
}