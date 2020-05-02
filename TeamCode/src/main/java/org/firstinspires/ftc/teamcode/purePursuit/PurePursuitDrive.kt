package org.firstinspires.ftc.teamcode.purePursuit

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import com.acmerobotics.roadrunner.localization.Localizer
import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.LUDecomposition
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import kotlin.math.*

const val TAU = Math.PI * 2

class PurePursuitDrive(val localizer: Localizer) {
    val waypoints: MutableList<Pose2d> = mutableListOf()
    val actions: MutableList<Pair<Int, () -> Unit>> = mutableListOf()

    private val lookAhead = 5.0 //Look Ahead Distance, 5 is arbitrary, depends on application and needs tuning, inches
    private val translationalTol = 0.5 //inches
    private val angularTol = Math.toRadians(1.0) // one degree angular tolerance

    private val axialCoeffs = PIDCoefficients(1.0, 0.0, 0.0) // no need for axial or lateral pid to be changed
    private val lateralCoeffs = PIDCoefficients(1.0, 0.0, 0.0) // ^
    private val headingCoeffs = PIDCoefficients(1.0, 0.0, 0.0) // change how every needed

    private val axialController = PIDFController(axialCoeffs)
    private val lateralController = PIDFController(lateralCoeffs)
    private val headingController = PIDFController(headingCoeffs)

    private val arcResolution = 6.0  // number of inches per point on arc
    private val speed = 50.0 //travel speed, inches
    private val threshold = 5.0 // distance at which to start reducing speed, inches

    init {
        headingController.setInputBounds(-Math.PI, Math.PI)
        axialController.update(0.0)
        lateralController.update(0.0)
        headingController.update(0.0)
    }

    fun followSync(drivetrain: DriveTrain, rel: Boolean = false) {

        var previous = 0
        var goal = 1

        val firstAction = actions.find { it.first == previous }?.second
        if (firstAction != null) firstAction()
        waypoints.add(0, localizer.poseEstimate)

        while (!Thread.currentThread().isInterrupted) {
            localizer.update()
            val currentPos = localizer.poseEstimate

            if (currentPos.vec() distTo waypoints.last().vec() < translationalTol &&
                    abs(currentPos.heading - waypoints.last().heading) < angularTol) {
                // at last waypoint
                break

            } else if (currentPos.vec() distTo waypoints[goal].vec() < translationalTol &&
                    abs(currentPos.heading - waypoints[goal].heading) < angularTol) {
                // go to next waypoint
                val action = actions.find { it.first == previous }?.second
                if (action != null) action()
                previous = goal
                goal += 1
                continue
            }

             val target = if (rel) {
                findGoalPointRelative(currentPos, this.waypoints[previous], this.waypoints[goal])
            } else {
                findGoalPointAbsolute(currentPos, this.waypoints[previous], this.waypoints[goal])
            }

            val multiplier = if (goal == waypoints.size - 1) {
                0.0
            } else {
                cos(waypoints[goal+1].vec() - waypoints[goal].vec()
                        angleBetween waypoints[goal].vec() - waypoints[previous].vec()) // calculate how similar the
            }

            val wheelVel = getWheelVelocityFromTarget(target, currentPos, waypoints[goal].vec() distTo currentPos.vec(), multiplier,
                    calculateTRelative(currentPos.vec(), waypoints[previous].vec(), waypoints[goal].vec()) )

            drivetrain.start(DriveTrain.Square(wheelVel[3], wheelVel[2], wheelVel[0], wheelVel[1]))

//            drivetrain.start(DriveTrain.Vector(robotVelocity.x, robotVelocity.y, -robotVelocity.heading).speeds()) // negative heading because the drive train module goes right on positive omega when real math would go left
        }
        this.waypoints.clear()
    }

    fun addPoint(point: Pose2d):PurePursuitDrive {
        waypoints.add(point)
        return this
    }


    fun addPoint(x: Double, y: Double, heading: Double): PurePursuitDrive {
        waypoints.add(Pose2d(x, y, heading))
        return this
    }

    fun addArc(start: Pose2d, mid: Vector2d, end: Pose2d): PurePursuitDrive {
        val denomMat = Array2DRowRealMatrix(arrayOf(
                doubleArrayOf(start.x, start.y, 1.0),
                doubleArrayOf(mid.x, mid.y, 1.0),
                doubleArrayOf(end.x, end.y, 1.0)
        ))
        if (LUDecomposition(denomMat).determinant == 0.0) {
            this.addPoint(Pose2d(mid, lerpAngle(start.heading, end.heading, 0.5))).addPoint(end)
            return this
        }
        val segment = ArcSegment.fromThreePoints(start.vec(), mid, end.vec())
        val numOfPoints = segment.length() / arcResolution
        for (i in 0..numOfPoints.toInt()) {
            val t = i / numOfPoints
            val vector = segment.r(t)
            waypoints.add(Pose2d(vector, lerp(start.heading, end.heading, t)))
        }
        return this
    }

    fun addArc(mid: Vector2d, end: Pose2d): PurePursuitDrive {
        return if (waypoints.size > 0) {
            addArc(waypoints.last(), mid, end)
        } else {
            addArc(localizer.poseEstimate, mid, end)
        }
    }

    fun addTurn(theta: Double): PurePursuitDrive {
        val last = waypoints.last()
        waypoints.add(Pose2d(last.vec(), last.heading + theta))
        return this
    }

    fun addAction(action : () -> Unit): PurePursuitDrive {
        actions.add(Pair(waypoints.size, action))
        return this
    }

    internal fun getWheelVelocityFromTarget(target:Pose2d, currentPos:Pose2d, length:Double, speedMultiplier: Double, t:Double): List<Double> {
        print(length)
        var error = Kinematics.calculatePoseError(target, currentPos)
        error = error.div(error.vec().norm()) // turn error into direction
        val adjustedSpeed: Double = if (speedMultiplier > 0) {
            val final = speed * speedMultiplier
            lerp(speed, final, t)
        } else {
            speed * length / threshold
        }

        val robotVelocity= getCorrectionVelocity(error).times(adjustedSpeed).times(0.0254) // to appropriate speed and meters as unit

        var wheelVel = MecanumKinematics.robotToWheelVelocities(robotVelocity, constants.trackwidth * 0.0254, constants.wheelBase * 0.0254)
                .map {it*constants.WheelVelToPowerConst}


        val wheelCopy = wheelVel.map {abs(it)}
        if (wheelCopy.max()!! > 1.0) {
            wheelVel = wheelVel.map {it/wheelCopy.max()!!}
        }
        return wheelVel
    }

    fun findGoalPointAbsolute(currentPose: Pose2d, start: Pose2d, end: Pose2d): Pose2d {
        if (currentPose.vec() distTo end.vec() < lookAhead) {
            return end
        }
        val t = calculateT(currentPose.vec(), start.vec(), end.vec())
        return getPointfromT(t, start, end)
    }


    internal fun findGoalPointRelative(currentPose: Pose2d, start: Pose2d, end: Pose2d): Pose2d {
        if (currentPose.vec() distTo end.vec() < lookAhead) {
            return end
        }
        val t = calculateTRelative(currentPose.vec(), start.vec(), end.vec())
        return getPointfromT(t, start, end)
    }
    internal fun getCorrectionVelocity(poseError: Pose2d): Pose2d {
        axialController.targetPosition = poseError.y
        lateralController.targetPosition = poseError.x
        headingController.targetPosition = poseError.heading
        val axialCorrection = axialController.update(0.0)
        val lateralCorrection = lateralController.update(0.0)
        val headingCorrection = headingController.update(0.0)
        return Pose2d(lateralCorrection, axialCorrection, headingCorrection)
    }

    internal fun calculateT(currentVec: Vector2d, start: Vector2d, end: Vector2d): Double {
        val d = end.minus(start)
        val f = start.minus(currentVec)
        val o = d.dot(f)

        val v = end.minus(start)
        val u = start.minus(currentVec)


        val discriminant = d.dot(d) * (lookAhead * lookAhead - f.dot(f)) + o * o

        return if (discriminant < 0) {
            // line segment to point as the look ahead circle doesnt intersect
            val t = -v.dot(u) / u.dot(u)
            limit(t, 0.0, 1.0)
        } else {
            val t = (sqrt(discriminant) - o) / d.dot(d)
            limit(t, 0.0, 1.0)
        }
    }

    internal fun calculateTRelative(currentVec: Vector2d, start: Vector2d, end: Vector2d): Double {
        val v = end.minus(start)
        val u = start.minus(currentVec)

        val t = -v.dot(u) / u.dot(u)
        val lineT= limit(t, 0.0, 1.0)
        val length = end.minus(start).norm()
        return max(lineT + lookAhead/length, 1.0)
    }

    internal fun limit(value: Double, min: Double, max: Double): Double {
        return max(min, min(value, max))
    }

    internal fun getPointfromT(t: Double, A: Pose2d, B: Pose2d): Pose2d {
        val x = lerp(A.x, B.x, t)
        val y = lerp(A.y, B.y, t)
        val heading = lerpAngle(A.heading, B.heading, t)
        return Pose2d(x, y, heading)
    }
}

open class ArcSegment(private val center: Vector2d, private val radius: Double, private val startAngle: Double, private val endAngle: Double) {

    companion object {
        fun fromThreePoints(ptBegin: Vector2d, ptMid: Vector2d, ptEnd: Vector2d): ArcSegment {
            val denomMat = Array2DRowRealMatrix(arrayOf(
                    doubleArrayOf(ptBegin.x, ptBegin.y, 1.0),
                    doubleArrayOf(ptMid.x, ptMid.y, 1.0),
                    doubleArrayOf(ptEnd.x, ptEnd.y, 1.0)
            ))
            val hNum = Array2DRowRealMatrix(arrayOf(
                    doubleArrayOf(ptBegin.dot(ptBegin), ptBegin.y, 1.0),
                    doubleArrayOf(ptMid.dot(ptMid), ptMid.y, 1.0),
                    doubleArrayOf(ptEnd.dot(ptEnd), ptEnd.y, 1.0)))
            val denom = (2 * LUDecomposition(denomMat).determinant)
            val h = LUDecomposition(hNum).determinant / denom

            val kNum = Array2DRowRealMatrix(arrayOf(
                    doubleArrayOf(ptBegin.x, ptBegin.dot(ptBegin), 1.0),
                    doubleArrayOf(ptMid.x, ptMid.dot(ptMid), 1.0),
                    doubleArrayOf(ptEnd.x, ptEnd.dot(ptEnd), 1.0)
            ))
            val k = LUDecomposition(kNum).determinant / denom

            val center = Vector2d(h, k)
            val radius = center distTo (ptBegin)
            val beginAngle = (ptBegin - center).angle()
            val endAngle = (ptEnd - center).angle()
            return ArcSegment(center, radius, beginAngle, endAngle)
        }
    }

    fun r(t: Double): Vector2d {
        val ang = angleFromT(t)
        return center + Vector2d(cos(ang), sin(ang)).times(radius)
    }

    private fun angleFromT(t: Double): Double {
        return lerp(startAngle, endAngle, t)
    }

    fun length(): Double {
        return (endAngle - startAngle).absoluteValue * radius
    }

}

fun lerp(a: Double, b: Double, t: Double): Double {
    return (b - a) * t + a
}

fun lerpAngle(a: Double, b: Double, t: Double): Double {

    return when {
        (b-a) > PI -> {
            lerp(b, a+TAU, t) % TAU
        }
        (b-a) < -PI -> {
            lerp(a, b+TAU, t) % TAU
        }
        else -> {
            lerp(a, b, t) % TAU
        }
    }
}