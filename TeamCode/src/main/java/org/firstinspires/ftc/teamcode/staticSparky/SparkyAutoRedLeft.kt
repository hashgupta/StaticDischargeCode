package org.firstinspires.ftc.teamcode.staticSparky

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Controllers.shootingGoal
import org.firstinspires.ftc.teamcode.Positions
import org.firstinspires.ftc.teamcode.pipelines.RingPipeline
import org.firstinspires.ftc.teamcode.robotConfigs.RobotBase
import org.firstinspires.ftc.teamcode.robotConfigs.SparkyV2Robot
import kotlin.math.PI


@Autonomous(name = "SparkyAutoRedLeft", group = "StaticDischarge")
class SparkyAutoRedLeft : GenericOpModeBase() {

    lateinit var robot: SparkyV2Robot
    val shootingPositionHighGoal = Pose2d(x = -0.1 * TILE_LENGTH, y = Positions.highGoalRed.y - 10.0, PI - Math.toRadians(2.0))
    val shootingPositionPowerShots = Vector2d(x = -0.20 * TILE_LENGTH, y = Positions.highGoalRed.y)
    val powerShotAngleAdjustment = Math.toRadians(-3.0)

    override fun runOpMode() {
        // UNCOMMENT THIS IF SOUNDS ARE NEEDED

        /*
        **************
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

        //start button has been hit
        // main auto code begins
        stopCV()


        // WARM UP SHOOTER EARLY
        robot.pursuiter.action { robot.shooter.aimShooter(Pose2d(-0.20 * TILE_LENGTH, Positions.powerFarRed.y, PI), Positions.powerFarRed) }

        // INTERMEDIATE POINT SO WE DON'T HIT RING STACK
        robot.pursuiter.move(-0.75 * TILE_LENGTH, -0.75 * TILE_LENGTH, 0.0)


        /* HIGH GOAL */

        //shoots 3 rings

        shootHighGoals(robot, shootingPositionHighGoal, 3)
        robot.pursuiter.action {
            robot.shooter.stopWheel()
        }

        /* FIRST WOBBLE */
        dropFirstWobble(robot, goalZone)


        //split path based on number of rings in stack
        // 0 or 4 -> stop the shooter and continue
        // 1 -> intake and shoot it, then stop the shooter
        when (analysis) {
            RingPipeline.RingPosition.ONE -> {

                robot.pursuiter.action {
                    robot.intake.start(-0.9)
                    robot.roller.start(0.9)
                }

                robot.pursuiter.spline(end = Pose2d(-TILE_LENGTH - 2, -TILE_LENGTH * 1.7, PI), endTanAngle = PI, startTanAngle = PI)

                robot.pursuiter.action {
                    sleep(750)
                    robot.intake.start(0.0)
                    robot.roller.start(0.0)
                }

                getSecondWobble(robot)

                shootHighGoals(robot, shootingPositionHighGoal, 1)

                robot.pursuiter.action {
                    robot.shooter.stopWheel()
                }

                dropSecondWobble(robot, goalZone)
            }
//            RingPipeline.RingPosition.FOUR -> {

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

//                shootHighGoals(robot, shootingPositionHighGoal, 3)


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
//                robot.pursuiter.turnTo(robot.shooter.turningTarget(robot.pursuiter.waypoints.last().end.vec(), Positions.highGoalRed))
//                shootHighGoals(robot, shootingPositionHighGoal, 1)
//            }
            else -> {
                /* SECOND WOBBLE */
                getSecondWobble(robot)
                dropSecondWobble(robot, goalZone)
            }
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

/* POWER SHOTS */
fun shootPowerShots(robot: SparkyV2Robot, shootingPosition: Vector2d, angleAdjustment: Double) {
        robot.pursuiter
                .move(Pose2d(shootingPosition, angleAdjustment + robot.shooter.turningTarget(shootingPosition, Positions.powerNearRed)))
                .action {
                    robot.shooter.aimShooter(robot.localizer.poseEstimate,
                            Positions.powerNearRed)
                    Thread.sleep(500)
                    robot.shooter.shoot()
                }

        robot.pursuiter
                .turnTo(angleAdjustment + robot.shooter.turningTarget(shootingPosition, Positions.powerMidRed))
                .action {
                    robot.shooter.aimShooter(robot.localizer.poseEstimate,
                            Positions.powerMidRed)
                    Thread.sleep(500)
                    robot.shooter.shoot()
                }

        robot.pursuiter
                .turnTo(angleAdjustment + robot.shooter.turningTarget(shootingPosition, Positions.powerFarRed))
                .action {
                    robot.shooter.aimShooter(robot.localizer.poseEstimate,
                            Positions.powerFarRed)
                    Thread.sleep(500)
                    robot.shooter.shoot()
                }
}

fun shootHighGoals(robot: SparkyV2Robot, shootingPosition: Pose2d, rings: Int) {
    if (rings == 0) {
        return
    }
    robot.pursuiter.move(shootingPosition)

    robot.pursuiter.action {
        robot.shooter.aimShooter(Pose2d(0.0, 0.0, 0.0), shootingGoal(75.0, 0.0, 34.8))
        // first ring
        Thread.sleep(500)
        robot.shooter.shoot()
        // all other rings
        for (i in 1 until rings) {
            Thread.sleep(150)
            robot.shooter.shoot()
        }
    }
}

fun dropFirstWobble(robot: SparkyV2Robot, goalZone:Pose2d) {
    robot.pursuiter
            .move(goalZone + Pose2d(-2.0, 5.0, -PI / 4))
            .action { robot.arm.dropAuto() }
}

fun getSecondWobble(robot: SparkyV2Robot) {
    robot.pursuiter
            .move(-1 * GenericOpModeBase.TILE_LENGTH, -2.4 * GenericOpModeBase.TILE_LENGTH, PI)
            .action { robot.arm.run(0.75)
                Thread.sleep(300)
                robot.arm.run(0.0)
                robot.pursuiter.runSpeed *= 0.5}

    robot.pursuiter
            .move(-1.6 * GenericOpModeBase.TILE_LENGTH, -2.4 * GenericOpModeBase.TILE_LENGTH, PI)
            .action {
                robot.arm.grabAuto()
                robot.pursuiter.runSpeed *= 2.0
                //buck it to the target zone
            }
}

fun dropSecondWobble(robot: SparkyV2Robot, goalZone: Pose2d) {
    robot.pursuiter
            .move(goalZone + Pose2d(-15.0, 2.0, -Math.toRadians((10.0))))
            .action {
                robot.arm.dropTele()
                Thread.sleep(1000)
            }
            .relative(0.0, -10.0, 0.0)
}