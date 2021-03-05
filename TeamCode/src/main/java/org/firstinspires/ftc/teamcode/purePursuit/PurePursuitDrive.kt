package org.firstinspires.ftc.teamcode.purePursuit

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.util.epsilonEquals
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max


//@Deprecated(message = "Don't use this class, it is old and has not been properly tested. None of the important changes have been added to make this functional. Please just use FastPurePursuit class for your movement needs")
@Config
class PurePursuitDrive(val localizer: Localizer) {
    val waypoints: MutableList<Path> = mutableListOf()
    val actions: MutableList<Pair<Int, () -> Unit>> = mutableListOf()

    private val lookAhead = 5.0 //Look Ahead Distance, 5 is arbitrary, depends on application and needs tuning, inches

    private val translationalTol = 1.0 //inches
    private val angularTol = Math.toRadians(1.0) // one degree angular tolerance

    private val translationalCoeffs: PIDCoefficients = PIDCoefficients(2.0)
    private val headingCoeffs: PIDCoefficients = PIDCoefficients(2.0)

    private val kV = 0.1
    private val kStatic = 0.1

    private val axialController = PIDFController(translationalCoeffs, kV = kV, kStatic = kStatic)
    private val lateralController = PIDFController(translationalCoeffs, kV = kV, kStatic = kStatic)
    private val headingController = PIDFController(headingCoeffs, kV = kV, kStatic = kStatic)

    private val speed = 50.0 //travel speed, inches
    private val threshold = 10.0 // distance at which to start reducing speed, inches

    init {
        axialController.update(0.0)
        lateralController.update(0.0)
        headingController.update(0.0)
    }

    fun followSync(drivetrain: DriveTrain) {

        var index = 0

        runAction(0)

        while (!Thread.currentThread().isInterrupted) {
            localizer.update()


            val currentPos = localizer.poseEstimate

            val currentPath = waypoints[index]

            val speedMultiplier = getSpeedMultiplier(index)

            val translationErrorMargin = lerp(translationalTol, translationalTol * 5, speedMultiplier)

            if (currentPos.vec() distTo waypoints[index].end.vec() < translationErrorMargin &&
                    abs(currentPos.heading - waypoints[index].end.heading) < angularTol) {
                // go to next waypoint
                runAction(index)
                if (index == waypoints.size-1) {
                    break
                } else {
                    index += 1
                    continue
                }
            }
            val target = findGoalPointRelative(currentPos, currentPath)

            val wheelVel = getWheelVelocityFromTarget(target = target, currentPos = currentPos, path = currentPath, index = index)

            drivetrain.start(DriveTrain.Square(wheelVel[3], wheelVel[2], wheelVel[0], wheelVel[1]))
        }
        this.waypoints.clear()
    }

    fun addPoint(point: Pose2d):PurePursuitDrive {
        if (waypoints.size > 0) {
            waypoints.add(LinearPath(waypoints.last().end, point))
        } else {
            waypoints.add(LinearPath(localizer.poseEstimate, point))
        }
        return this
    }

    fun addPoint(x: Double, y: Double, heading: Double): PurePursuitDrive {
        return addPoint(Pose2d(x, y, heading))
    }

    fun setPose(start: Pose2d): PurePursuitDrive{
        localizer.poseEstimate = start
        return this
    }

    fun addTurn(theta: Double): PurePursuitDrive {
        val last = waypoints.last()
        val next = Pose2d(last.end.vec(), last.end.heading + theta)
        waypoints.add(LinearPath(last.end, next))
        return this
    }

    fun addAction(action : () -> Unit): PurePursuitDrive {
        actions.add(Pair(waypoints.size, action))
        return this
    }

    fun runAction(index:Int) {
        val action = actions.find { it.first == index }?.second
        if (action != null) action()
    }

    fun estimateTime(start: Pose2d):Double{
        var time = 0.0
        val copyWaypoints = waypoints.toMutableList()
        copyWaypoints.add(0, LinearPath(start, waypoints[0].start))
        for ((index, waypoint) in copyWaypoints.withIndex()) {
            if (index == copyWaypoints.size - 1) {
                continue
            }
            val path = (waypoint.end - waypoint.start).vec()

            if (path.norm() epsilonEquals  0.0) {
                continue
            }

            // cant use speed multiplier function because this set of waypoints is different than the waypoints stored in the object
            val speedMultiplier = if (index == copyWaypoints.size - 2 ) {
                0.0
            } else {
                cos( ( (copyWaypoints[index+1].end - copyWaypoints[index+1].start).vec()
                        angleBetween path ) ) // calculate how similar the next path and the current are
            }


            if (speedMultiplier <= 0.0) {
                time += (path.norm() - threshold) / (speed)
                time += threshold / (speed * 0.55)
            } else {
                time += path.norm() / ((2 * speed) / (1 + speedMultiplier))
            }
        }
        return time
    }

    fun getWheelVelocityFromTarget(target:Pose2d, currentPos:Pose2d, path:Path, index:Int): List<Double> {

        val speedMultiplier = getSpeedMultiplier(index)

        val left = (path.end.vec() - currentPos.vec()).norm()

        val pathVector = (path.end - path.start).vec()

        val t = path.findClosestT(currentPos)

        val error = Kinematics.calculatePoseError(target, currentPos)

        val adjustedSpeed: Double = if (speedMultiplier > 0.1) {
            val final = speed * speedMultiplier
            lerp(speed, final, t)
        } else {
            if (left > threshold)
                speed
            else {
                max(speed * left / threshold, 0.1 * speed)

            }
        }

        val targetVelocity = Pose2d(pathVector.div(pathVector.norm()).times(adjustedSpeed), 0.0) // turn speed into velocity in direction of goal

        val power = errorToPower(error, targetVelocity)

        var wheelPow = MecanumKinematics.robotToWheelVelocities(power, Constants.trackwidth, Constants.wheelBase, lateralMultiplier = 1.0)


        val wheelCopy = wheelPow.map {abs(it)}

        if (wheelCopy.maxOrNull() != null && wheelCopy.maxOrNull()!! > 1) {
            wheelPow = wheelPow.map {it/ wheelCopy.maxOrNull()!!}
        }

        return wheelPow
    }


    fun findGoalPointRelative(currentPose: Pose2d, path:Path): Pose2d {
        if (currentPose.vec() distTo path.end.vec() < lookAhead) {
            return path.end
        }
        val t = path.findClosestT(currentPose)
        return path.getPointfromT(t)
    }

    fun errorToPower(poseError: Pose2d, targetVelocity : Pose2d?): Pose2d {
        axialController.targetPosition = poseError.x
        lateralController.targetPosition = poseError.y
        headingController.targetPosition = poseError.heading

        if (targetVelocity != null) {
            axialController.targetVelocity = targetVelocity.x
            lateralController.targetVelocity = targetVelocity.y
            headingController.targetVelocity = targetVelocity.heading
        }

        // note: feedforward is processed at the wheel level
        val axialCorrection = axialController.update(0.0)
        val lateralCorrection = lateralController.update(0.0)
        val headingCorrection = headingController.update(0.0)

        return Pose2d(
                axialCorrection,
                lateralCorrection,
                headingCorrection
        )
    }

    fun getSpeedMultiplier(index: Int):Double {
        return if (index == waypoints.size - 1) {
            0.0
        } else {
            cos( ( (waypoints[index+1].end.vec() - waypoints[index+1].start.vec())
                    angleBetween (waypoints[index].end.vec() - waypoints[index].start.vec()) ) ) // calculate how similar the next path and the current are
        }
    }
}
