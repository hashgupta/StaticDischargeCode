package org.firstinspires.ftc.teamcode.purePursuit

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.util.epsilonEquals
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import kotlin.math.abs
import kotlin.math.sign



//NOTE just mecanum, if tank is needed look at SimplePathPurePursuit
//@Deprecated(message = "Don't use this class, it is old and has not been properly tested. None of the important changes have been added to make this functional. Please just use FastPurePursuit class for your movement needs")
@Config
class SimplePurePursuit(val localizer: Localizer, val startPose:Pose2d) {
    val waypoints: MutableList<Pose2d> = mutableListOf()
    val actions: MutableList<Pair<Int, () -> Unit>> = mutableListOf()

    private val lookAhead = 10.0 //Look Ahead Distance, 5 is arbitrary, depends on application and needs tuning, inches

    private val translationalTol = 0.2 //inches
    private val angularTol = Math.toRadians(1.0) // one degree angular tolerance
    private val kStatic = 0.15

    private val translationalCoeffs: PIDCoefficients = PIDCoefficients(0.05)
    private val headingCoeffs: PIDCoefficients = PIDCoefficients(0.05)

    private val axialController = PIDFController(translationalCoeffs)
    private val lateralController = PIDFController(translationalCoeffs)
    private val headingController = PIDFController(headingCoeffs)

    init {
        axialController.update(0.0)
        lateralController.update(0.0)
        headingController.update(0.0)
    }

    fun followSync(drivetrain: DriveTrain) {

        var previous = 0
        var goal = 1

        runAction(0)

        waypoints.add(0, startPose)

        while (!Thread.currentThread().isInterrupted) {
            localizer.update()

            val currentPos = localizer.poseEstimate

            if (currentPos.vec() distTo waypoints.last().vec() < translationalTol &&
                    abs(currentPos.heading - waypoints.last().heading) < angularTol) {
                // at last waypoint
                runAction(goal)
                break
            }

            if (currentPos.vec() distTo waypoints[goal].vec() < translationalTol &&
                    abs(currentPos.heading - waypoints[goal].heading) < angularTol) {
                // go to next waypoint
                runAction(goal)
                previous = goal
                goal += 1
                continue
            }
            val target = findGoalPointRelative(currentPos, this.waypoints[previous], this.waypoints[goal])

            val wheelVel = getWheelVelocityFromTarget(target = target, currentPos = currentPos)

            drivetrain.start(DriveTrain.Square(wheelVel[3], wheelVel[2], wheelVel[0], wheelVel[1]))
        }
        this.waypoints.clear()
    }

    fun addPoint(point: Pose2d):SimplePurePursuit {
        waypoints.add(point)
        return this
    }

    fun runAction(index:Int) {
        val action = actions.find { it.first == index }?.second
        if (action != null) action()
    }


    fun addPoint(x: Double, y: Double, heading: Double): SimplePurePursuit {
        waypoints.add(Pose2d(x, y, heading))
        return this
    }

    fun setPose(start: Pose2d): SimplePurePursuit{
        localizer.poseEstimate = start
        return this
    }

    fun addTurn(theta: Double): SimplePurePursuit {
        val last = waypoints.last()
        waypoints.add(Pose2d(last.vec(), last.heading + theta))
        return this
    }

    fun addAction(action : () -> Unit): SimplePurePursuit {
        actions.add(Pair(waypoints.size, action))
        return this
    }

    fun getWheelVelocityFromTarget(target:Pose2d, currentPos:Pose2d): List<Double> {

        val error = Kinematics.calculatePoseError(target, currentPos)

        val velocity = errorToPower(error)

        var wheelPow = MecanumKinematics.robotToWheelVelocities(velocity, Constants.trackwidth, Constants.wheelBase, lateralMultiplier = 1.0)

        wheelPow = wheelPow.map { it + sign(it) * kStatic }

        val wheelCopy = wheelPow.map {abs(it)}

        if (wheelCopy.maxOrNull() != null && wheelCopy.maxOrNull()!! > 1) {
            wheelPow = wheelPow.map {it/ wheelCopy.maxOrNull()!!}
        }

        return wheelPow
    }


    fun findGoalPointRelative(currentPose: Pose2d, start: Pose2d, end: Pose2d): Pose2d {
        if (currentPose.vec() distTo end.vec() < lookAhead) {
            return end
        }

        val t = calculateTRelative(currentPose.vec(), start.vec(), end.vec())
        val length = end.vec().minus(start.vec()).norm()

        return getPointfromT(t + lookAhead/length, start, end)
    }

    fun errorToPower(poseError: Pose2d): Pose2d {
        axialController.targetPosition = poseError.x
        lateralController.targetPosition = poseError.y
        headingController.targetPosition = poseError.heading


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

    fun calculateTRelative(currentVec: Vector2d, start: Vector2d, end: Vector2d): Double {
        val v = end.minus(start)
        val u = start.minus(currentVec)

        val t = if (u.dot(u) epsilonEquals 0.0) {
            0.0
        } else {
            -v.dot(u) / u.dot(u)
        }

        return limit(t, 0.0, 1.0)
    }

    fun getPointfromT(t: Double, A: Pose2d, B: Pose2d): Pose2d {
        val x = lerp(A.x, B.x, t)
        val y = lerp(A.y, B.y, t)
        val heading = lerpAngle(A.heading, B.heading, t)
        return Pose2d(x, y, heading)
    }
}