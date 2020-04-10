package org.firstinspires.ftc.teamcode.purePursuit

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.localization.Localizer
import org.firstinspires.ftc.teamcode.hardware.DriveTrain
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.LUDecomposition

class PurePursuitDrive(val localizer: Localizer) {
    val waypoints: MutableList<Pose2d> = mutableListOf()

    private val lookAhead = 5 //Look Ahead Distance, 5 is arbitrary, depends on application and needs tuning
    private val translationalTol = 0.5
    private val angularTol = Math.toRadians(1.0) // one degree angular tolerance
    private val axialCoeffs = PIDCoefficients(0.0, 0.0, 0.0)
    private val lateralCoeffs = PIDCoefficients(0.0, 0.0, 0.0)
    private val headingCoeffs = PIDCoefficients(0.0, 0.0, 0.0)
    private val axialController = PIDFController(axialCoeffs)
    private val lateralController = PIDFController(lateralCoeffs)
    private val headingController = PIDFController(headingCoeffs)
    private val ArcResolution = 50.0


    fun AutofollowSync(drivetrain: DriveTrain) {

        var previous = 0
        var goal = 1
        var done = false
        while (!Thread.currentThread().isInterrupted) {
            localizer.update()
            val currentPos = localizer.poseEstimate

            if (currentPos.vec() distTo waypoints.last().vec() < translationalTol &&
                    abs(currentPos.heading - waypoints.last().heading) < angularTol) {
                break
            } else if (currentPos.vec() distTo waypoints[goal].vec() < translationalTol &&
                    abs(currentPos.heading - waypoints[goal].heading) < angularTol) {
                previous = goal
                goal += 1
                continue
            }

            val target = findGoalPoint(currentPos, this.waypoints[previous], this.waypoints[goal])
            val error = Kinematics.calculatePoseError(target, currentPos)
            val vel = getCorrectionVelocity(error)
            val robotVelocity = Kinematics.fieldToRobotVelocity(currentPos, vel)

            drivetrain.start(DriveTrain.Vector(robotVelocity.x, robotVelocity.y, -robotVelocity.heading).speeds()) // negative heading because the drive train module goes right on positive omega when real math would go left
        }
        this.waypoints.clear()
    }

    fun addPoint(point: Pose2d): PurePursuitDrive {
        waypoints.add(point)
        return this
    }

    fun addPoint(x: Double, y: Double, heading: Double): PurePursuitDrive {
        waypoints.add(Pose2d(x, y, heading))
        return this
    }

    fun addArc(start: Pose2d, mid: Vector2d, end: Pose2d): PurePursuitDrive {
        val segment = ArcSegment.fromThreePoints(start.vec(), mid, end.vec())
        for (i in 0..ArcResolution.toInt()) {
            val t = i / ArcResolution
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

    private fun findGoalPoint(currentPose: Pose2d, start: Pose2d, end: Pose2d): Pose2d {
        if (currentPose.vec() distTo end.vec() < lookAhead) {
            return end
        }
        val t = calculateT(currentPose.vec(), start.vec(), end.vec())
        return getPointfromT(t, start, end)
    }

    private fun getCorrectionVelocity(poseError: Pose2d): Pose2d {
        axialController.targetPosition = poseError.x
        lateralController.targetPosition = poseError.y
        headingController.targetPosition = poseError.heading

        val axialCorrection = axialController.update(0.0)
        val lateralCorrection = lateralController.update(0.0)
        val headingCorrection = headingController.update(0.0)

        return Pose2d(axialCorrection, lateralCorrection, headingCorrection)

    }

    private fun calculateT(currentVec: Vector2d, start: Vector2d, end: Vector2d): Double {
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

    private fun limit(value: Double, min: Double, max: Double): Double {
        return Math.max(min, Math.min(value, max))
    }

    private fun getPointfromT(t: Double, A: Pose2d, B: Pose2d): Pose2d {
        val x = lerp(A.x, B.x, t)
        val y = lerp(A.y, B.y, t)
        val heading = lerp(A.heading, B.heading, t)
        return Pose2d(x, y, heading)
    }
}

open class ArcSegment(private val center: Vector2d, private val radius: Double, private val startAngle: Double, private val endAngle: Double) {

    companion object {
        fun fromThreePoints(ptBegin: Vector2d, ptMid: Vector2d, ptEnd: Vector2d): ArcSegment {
            val hNum = Array2DRowRealMatrix(arrayOf(
                    doubleArrayOf(ptBegin.dot(ptBegin), ptBegin.y, 1.0),
                    doubleArrayOf(ptMid.dot(ptMid), ptMid.y, 1.0),
                    doubleArrayOf(ptEnd.dot(ptEnd), ptEnd.y, 1.0)))
            val denomMat = Array2DRowRealMatrix(arrayOf(
                    doubleArrayOf(ptBegin.x, ptBegin.y, 1.0),
                    doubleArrayOf(ptMid.x, ptMid.y, 1.0),
                    doubleArrayOf(ptEnd.x, ptEnd.y, 1.0)
            ))
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

}

fun lerp(a: Double, b: Double, t: Double): Double {
    return (b - a) * t + a
}