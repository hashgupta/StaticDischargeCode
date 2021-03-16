package org.firstinspires.ftc.teamcode.staticSparky

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Positions
import org.firstinspires.ftc.teamcode.pipelines.RingPipeline
import org.firstinspires.ftc.teamcode.robotConfigs.SparkyV2Robot
import kotlin.math.PI


@Autonomous(name = "SparkyAutoRedLeft", group = "StaticDischarge")
class SparkyAutoRedLeft : GenericOpModeBase() {

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



        robot = SparkyV2Robot(hardwareMap, telemetry) { opModeIsActive() && !isStopRequested }
        robot.pursuiter.startAt(Positions.startLeftRed)
        initCV(Side.Right)
        startCV()
        var analysis: RingPipeline.RingPosition = RingPipeline.RingPosition.NONE
        while (!isStarted) {
            if (isStopRequested) {
                return
            }

            analysis = (pipeline as RingPipeline).position()
            telemetry.addData("analysis",analysis)
            telemetry.update()
        }

        //start button has been hit
        // main auto code begins
        stopCV()




        //WARM UP SHOOTER EARLY
        robot.pursuiter.action { robot.shooter.aimShooter(Pose2d(-0.20* TILE_LENGTH, Positions.powerFarRed.y - 5, PI), Positions.powerFarRed) }

        // INTERMEDIATE POINT SO WE DON'T HIT RING STACK
        robot.pursuiter.move(-0.75* TILE_LENGTH, -0.5* TILE_LENGTH, PI)
        robot.pursuiter.action {}



        /* HIGH GOAL */

        //shoots 3 rings

        robot.pursuiter.move(Pose2d(x = -0.23* TILE_LENGTH, y = Positions.highGoalRed.y - 5, heading = robot.shooter.turningTarget(Vector2d(-0.0* TILE_LENGTH, Positions.highGoalRed.y), Positions.highGoalRed)))

        robot.pursuiter.action {
            robot.shooter.aimShooter(robot.localizer.poseEstimate,
                Positions.highGoalRed);
                sleep(1000);
                robot.shooter.shoot()
                sleep(500);
                robot.shooter.shoot();
                sleep(500);
                robot.shooter.shoot();
        }


        //split path based on number of rings in stack
        // 0 or 4 -> stop the shooter and continue
        // 1 -> intake and shoot it, then stop the shooter


        if (analysis == RingPipeline.RingPosition.ONE) {
            robot.pursuiter.action {
                robot.intake.start(-0.6)
                robot.roller.start(0.7)
            }
            robot.pursuiter.move(-TILE_LENGTH, -TILE_LENGTH*1.5, PI)
            robot.pursuiter.action {
                sleep(500)
                robot.intake.start(0.0)
                robot.roller.start(0.0)
            }

            robot.pursuiter.move(Pose2d(x = -0.23* TILE_LENGTH, y = Positions.highGoalRed.y - 5, heading = robot.shooter.turningTarget(Vector2d(-0.0* TILE_LENGTH, Positions.highGoalRed.y), Positions.highGoalRed)))

            robot.pursuiter.action {
                robot.shooter.aimShooter(robot.localizer.poseEstimate,
                        Positions.highGoalRed)
                sleep(750)
                robot.shooter.shoot()
                robot.shooter.stopWheel()
            }
        } else {
            robot.pursuiter.action {
                robot.shooter.stopWheel()
            }
        }



        /* POWER SHOTS */


//        robot.pursuiter
//                .move(Pose2d(-0.20* TILE_LENGTH, Positions.powerFarRed.y - 4, PI + Math.toRadians(-2.0)))
//                .action {
//            robot.shooter.aimShooter(robot.localizer.poseEstimate,
//                    Positions.powerFarRed)
//            sleep(250)
//            robot.shooter.shoot()
//        }
//
//        robot.pursuiter
//                .move(Pose2d(-0.20* TILE_LENGTH, Positions.powerMidRed.y - 4, PI))
//                .action {
//            robot.shooter.aimShooter(robot.localizer.poseEstimate,
//                Positions.powerMidRed)
//            sleep(250)
//            robot.shooter.shoot()
//        }
//
//        robot.pursuiter
//                .move(-0.20* TILE_LENGTH, Positions.powerNearRed.y - 5, PI)
//                .action {
//            robot.shooter.aimShooter(robot.localizer.poseEstimate,
//                Positions.powerNearRed)
//            sleep(250)
//            robot.shooter.shoot()
//            robot.shooter.stopWheel()
//        }



        val goalZone = when (analysis) {
            RingPipeline.RingPosition.NONE -> {
                Pose2d(Positions.aZoneRed, 0.0)
            }
            RingPipeline.RingPosition.ONE -> {
                Pose2d(Positions.bZoneRed, 0.0)
            }
            else -> {
                Pose2d(Positions.cZoneRed, 0.0)
            }
        }



        /* FIRST WOBBLE */

        robot.pursuiter
                .move(goalZone + Pose2d(-8.0, 12.0, -PI/4))
                .action{ robot.arm.dropAuto() }




        /* SECOND WOBBLE */

//        robot.pursuiter
//                .move(-1* TILE_LENGTH,-2.25* TILE_LENGTH, PI)
//                .action {robot.pursuiter.runSpeed = 0.35}
//
//        robot.pursuiter
//                .move(-1.9 * TILE_LENGTH + 1.0, -2.22*TILE_LENGTH + 2.25, PI)
//                .action { robot.arm.grabAuto()
//                robot.pursuiter.runSpeed = 0.75}
//
//        robot.pursuiter
//                .move(goalZone + Pose2d(-22.0, 12.0, -Math.toRadians((10.0))))
//                .action { robot.arm.dropTele()
//                    sleep(500)
//                }



        /* PARK */

        robot.pursuiter.move(0.5 * TILE_LENGTH, -1* TILE_LENGTH, PI)



        // the entire auto program above is dynamically building the path and actions
        // the line below does all the activity
        robot.pursuiter.follow(robot.driveTrain, telemetry = telemetry)

        //record position for teleop
        robot.savePose()

    }
}