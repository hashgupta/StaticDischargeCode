package org.firstinspires.ftc.teamcode.staticSparky

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Controllers.shootingGoal
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
            telemetry.addData("analysis", analysis)
            telemetry.update()
        }

        //start button has been hit
        // main auto code begins
        stopCV()


        //WARM UP SHOOTER EARLY
        robot.pursuiter.action { robot.shooter.aimShooter(Pose2d(-0.20 * TILE_LENGTH, Positions.powerFarRed.y, PI), Positions.powerFarRed) }

        // INTERMEDIATE POINT SO WE DON'T HIT RING STACK
        robot.pursuiter.move(-0.75 * TILE_LENGTH, -0.75 * TILE_LENGTH, 0.0)


        /* HIGH GOAL */

        //shoots 3 rings

        val shootingPositionHighGoal = Pose2d(x = -0.1 * TILE_LENGTH, y = Positions.highGoalRed.y - 10.0, PI - Math.toRadians(2.0))
        val shootingPositionPowerShots = Vector2d(x = -0.20 * TILE_LENGTH, y = Positions.highGoalRed.y)
        val powerShotAngleAdjustment = Math.toRadians(-3.0)

        robot.pursuiter.move(shootingPositionHighGoal)

        robot.pursuiter.action {
            robot.shooter.aimShooter(Pose2d(0.0, 0.0, 0.0), shootingGoal(75.0, 0.0, 35.0))
                sleep(500)
                robot.shooter.shoot()
                sleep(150)
                robot.shooter.shoot()
                sleep(150)
                robot.shooter.shoot()
        }

        /* POWER SHOTS */
        // shoot from same spot as high goal


//        robot.pursuiter
//                .move(Pose2d(shootingPositionPowerShots, powerShotAngleAdjustment + robot.shooter.turningTarget(shootingPositionPowerShots, Positions.powerNearRed)))
//                .action {
//                    robot.shooter.aimShooter(robot.localizer.poseEstimate,
//                            Positions.powerNearRed)
//                    sleep(500)
//                    robot.shooter.shoot()
//                }
//
//        robot.pursuiter
//                .turnTo(powerShotAngleAdjustment + robot.shooter.turningTarget(shootingPositionPowerShots, Positions.powerMidRed))
//                .action {
//                    robot.shooter.aimShooter(robot.localizer.poseEstimate,
//                            Positions.powerMidRed)
//                    sleep(500)
//                    robot.shooter.shoot()
//                }
//
//        robot.pursuiter
//                .turnTo(powerShotAngleAdjustment + robot.shooter.turningTarget(shootingPositionPowerShots, Positions.powerFarRed))
//                .action {
//                    robot.shooter.aimShooter(robot.localizer.poseEstimate,
//                            Positions.powerFarRed)
//                    sleep(500)
//                    robot.shooter.shoot()
//                }


        //determine if we skip the wobble goal
//        val skipSecondWobble = !(analysis == RingPipeline.RingPosition.ONE)
        val skipSecondWobble = false


        //split path based on number of rings in stack
        // 0 or 4 -> stop the shooter and continue
        // 1 -> intake and shoot it, then stop the shooter
        when (analysis) {
            RingPipeline.RingPosition.ONE -> {

                robot.pursuiter.action {
                    robot.intake.start(-0.9)
                    robot.roller.start(0.9)
                }

                robot.pursuiter.spline(end = Pose2d(-TILE_LENGTH - 2, -TILE_LENGTH * 1.7, PI), endTanAngle = PI, startTanAngle = PI/2)

                robot.pursuiter.action {
                    sleep(750)
                    robot.intake.start(0.0)
                    robot.roller.start(0.0)
                }

                robot.pursuiter.move(shootingPositionHighGoal)

                robot.pursuiter.action {
                    robot.shooter.aimShooter(Pose2d(0.0, 0.0, 0.0), shootingGoal(75.0, 0.0, 35.0))
                    sleep(300)
                    robot.shooter.shoot()
                }
            }
            RingPipeline.RingPosition.FOUR -> {

                //knock over stack

//                robot.pursuiter.spline(end = Pose2d(-TILE_LENGTH + 5, -TILE_LENGTH * 1.6, PI), endTanAngle = PI)

//                robot.pursuiter.move(Pose2d(-TILE_LENGTH + 20, -TILE_LENGTH * 1.6, PI))
                // run intake backwards
//                robot.pursuiter.action {
//                    robot.intake.start(0.3)
//                    robot.roller.start(-0.3)
//                }
//                robot.pursuiter.spline(end = Pose2d(-TILE_LENGTH + 10, -TILE_LENGTH * 1.5, PI), endTanAngle = PI, startTanAngle = PI/2)
//
//                //start intake
//
//                robot.pursuiter.action {
//                        robot.pursuiter.runSpeed = 0.3
//                        robot.intake.start(-0.95)
//                        robot.roller.start(0.95)
//                }
//
//                //run over rings
//
//                robot.pursuiter.spline(Pose2d(-TILE_LENGTH-20, -TILE_LENGTH * 1.475, PI), PI)
//
//                //turn off intake
//
//                robot.pursuiter.action {
//                    sleep(700)
//                    robot.intake.start(0.0)
//                    robot.roller.start(0.0)
//                    robot.pursuiter.runSpeed = 0.75
//                }
//
//                //turn to shoot
//
////                robot.pursuiter.turnTo(robot.shooter.turningTarget(robot.pursuiter.waypoints.last().end.vec(), Positions.highGoalRed))
//                robot.pursuiter.move(Pose2d(shootingPositionHighGoal, heading = PI))
//
//                //FIRE!!
//                robot.pursuiter.action {
//                    robot.shooter.aimShooter(robot.localizer.poseEstimate,
//                            Positions.highGoalRed)
//                    sleep(750)
//                    robot.shooter.shoot()
//                    sleep(250)
//                    robot.shooter.shoot()
//                    sleep(250)
//                    robot.shooter.shoot()
//                }



                // same thing for last ring

//                robot.pursuiter.action {
//                        robot.intake.start(-0.9)
//                        robot.roller.start(0.9)
//                }
//
//                robot.pursuiter.spline(Pose2d(-TILE_LENGTH-20 , -TILE_LENGTH * 1.5, PI), endTanAngle = PI, startTanAngle = PI/2)
//
//                robot.pursuiter.action {
//                    sleep(500)
//                    robot.intake.start(0.0)
//                    robot.roller.start(0.0)
//                }
//
////                robot.pursuiter.turnTo(robot.shooter.turningTarget(robot.pursuiter.waypoints.last().end.vec(), Positions.highGoalRed))
//                robot.pursuiter.move(Pose2d(shootingPositionHighGoal, heading = PI))
//
//                //FIRE!!
//                robot.pursuiter.action {
//                    robot.shooter.aimShooter(robot.localizer.poseEstimate,
//                            Positions.highGoalRed)
//                    sleep(500)
//                    robot.shooter.shoot()
//                }

            }
            else -> {
            }
        }

        robot.pursuiter.action {
            robot.shooter.stopWheel()
        }


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
                .move(goalZone + Pose2d(-2.0, 5.0, -PI / 4))
                .action { robot.arm.dropAuto() }


//        /* SECOND WOBBLE */

        if (!skipSecondWobble) {

            robot.pursuiter
                    .move(-1 * TILE_LENGTH, -2.4 * TILE_LENGTH, PI)
                    .action { robot.arm.run(0.75)
                        Thread.sleep(300)
                        robot.arm.run(0.0)
                    robot.pursuiter.runSpeed *= 0.75}

            robot.pursuiter
                    .move(-1.6 * TILE_LENGTH, -2.4 * TILE_LENGTH, PI)
                    .action {
                        robot.arm.grabAuto()
                        robot.pursuiter.runSpeed *= 1.33
                        //buck it to the target zone
                    }

            robot.pursuiter
                    .move(goalZone + Pose2d(-15.0, 2.0, -Math.toRadians((10.0))))
                    .action {
                        robot.arm.dropTele()
                        sleep(1000)
                    }
                    .relative(0.0, -10.0, 0.0)
        }


        /* PARK */

        robot.pursuiter.move(0.5 * TILE_LENGTH, -1 * TILE_LENGTH, 0.0)


        // the entire auto program above is dynamically building the path and actions
        // the line below does all the activity
        robot.pursuiter.follow(robot.driveTrain, telemetry = telemetry)

        //record position for teleop
        robot.savePose()

    }
}