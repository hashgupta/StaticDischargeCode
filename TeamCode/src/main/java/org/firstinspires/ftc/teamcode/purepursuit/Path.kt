package org.firstinspires.ftc.teamcode.purepursuit

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.util.epsilonEquals
import kotlin.math.*

abstract class Path {
    abstract val start: Pose2d

    abstract val end: Pose2d

    abstract val length: Double

    abstract fun getPointfromT(t: Double): Pose2d

    abstract fun findClosestT(position: Pose2d): Double

    fun limit(value: Double, min: Double, max: Double): Double {
        return max(min, min(value, max))
    }

    override fun toString(): String {
        return "start: $start, end: $end, ${this.javaClass.simpleName}"
    }
}

class LinearPath(override val start: Pose2d, override val end: Pose2d) : Path() {
    override val length = end.vec() distTo start.vec()

    override fun getPointfromT(t: Double): Pose2d {
        val x = lerp(start.x, end.x, t)
        val y = lerp(start.y, end.y, t)
        val heading = lerpAngle(start.heading, end.heading, t)
        return Pose2d(x, y, heading)
    }

    override fun findClosestT(position: Pose2d): Double {
        val v = end.vec().minus(start.vec())

        if (v distTo Vector2d(0.0, 0.0) epsilonEquals 0.0) {
            throw Exception("cant have a linear path with same start and end x,y positions. Use turn path for this")
        }
        val c = position.vec() - start.vec()
        return limit((v.dot(c)) / (v.dot(v)), 0.0, 1.0)
    }
}

class TurnPath(override val start: Pose2d, override val end: Pose2d) : Path() {
    override val length: Double
        get() = abs(end.heading - start.heading) * 9.0

    override fun findClosestT(position: Pose2d): Double {
        return limit((position.heading - start.heading) / (end.heading - start.heading), 0.0, 1.0)

    }

    override fun getPointfromT(t: Double): Pose2d {
        val x = lerp(start.x, end.x, t)
        val y = lerp(start.y, end.y, t)
        val heading = lerpAngle(start.heading, end.heading, t)
        return Pose2d(x, y, heading)
    }
}

class CubicSplinePath(override val start: Pose2d, override val end: Pose2d, startTangent: Double, val endTangent: Double) : Path() {

    override val length: Double
    private var xCoeffs: Coeffs
    private var yCoeffs: Coeffs

    init {
        val derivMagStrong = 2 * (start.vec() distTo end.vec())
        val derivMagWeak = (start.vec() distTo end.vec())
        xCoeffs = calculateCoeffs(start.x, end.x, derivMagWeak * cos(startTangent), derivMagStrong * cos(endTangent))
        yCoeffs = calculateCoeffs(start.y, end.y, derivMagWeak * sin(startTangent), derivMagStrong * sin(endTangent))
        length = length(0.0, 1.0)
    }

    data class Coeffs(val a: Double, val b: Double, val c: Double, val d: Double)

    override fun getPointfromT(t: Double): Pose2d {
        val xPos = xCoeffs.a * (t * t * t) + xCoeffs.b * (t * t) + xCoeffs.c * t + xCoeffs.d

        val yPos = yCoeffs.a * (t * t * t) + yCoeffs.b * (t * t) + yCoeffs.c * t + yCoeffs.d

        val head = lerpAngle(start.heading, end.heading, t)

        return Pose2d(xPos, yPos, head)
    }

    private fun derivFromT(t: Double): Vector2d {
        val xPos = 3 * xCoeffs.a * (t * t) + 2 * xCoeffs.b * (t) + xCoeffs.c

        val yPos = 3 * yCoeffs.a * (t * t) + 2 * yCoeffs.b * (t) + yCoeffs.c

        return Vector2d(xPos, yPos)
    }

    private fun secondDerivFromT(t: Double): Vector2d {
        val xPos = 6 * xCoeffs.a * (t) + (2 * xCoeffs.b)

        val yPos = 6 * yCoeffs.a * (t) + (2 * yCoeffs.b)

        return Vector2d(xPos, yPos)
    }

    override fun findClosestT(position: Pose2d): Double {
        var minDist: Double = Double.MAX_VALUE
        var upperT: Double
        var lowerT: Double


        upperT = 1.0
        lowerT = 0.0


        var bestT = (upperT + lowerT) / 2.0
        var range = upperT - lowerT

        val numberOfSteps = 10

        for (q in 1..3) {
            for (i in 0..numberOfSteps) {
                val testT = lerp(lowerT, upperT, i / numberOfSteps.toDouble())
                val newDist = position.vec() distTo getPointfromT(testT).vec()
                if (newDist < minDist) {
                    minDist = newDist
                    bestT = testT
                }
            }


            upperT = limit(bestT + range / 10.0, 0.0, 1.0)
            lowerT = limit(bestT - range / 10.0, 0.0, 1.0)
            range = upperT - lowerT
        }


        return bestT
    }


    private fun calculateCoeffs(start: Double, end: Double, startDeri: Double, endDeri: Double): Coeffs {
        val c = startDeri
        val d = start
        val AplusB = end - c - d
        val a = endDeri - (2 * AplusB) - c
        val b = AplusB - a
        return Coeffs(a, b, c, d)
    }

    private fun approxLength(v1: Vector2d, v2: Vector2d, v3: Vector2d): Double {
        val w1 = (v2 - v1) * 2.0
        val w2 = (v2 - v3) * 2.0
        val det = w1.x * w2.y - w2.x * w1.y
        val chord = v1 distTo v3
        return if (det epsilonEquals 0.0) {
            chord
        } else {
            val x1 = v1.x * v1.x + v1.y * v1.y
            val x2 = v2.x * v2.x + v2.y * v2.y
            val x3 = v3.x * v3.x + v3.y * v3.y

            val y1 = x2 - x1
            val y2 = x2 - x3

            val origin = Vector2d(y1 * w2.y - y2 * w1.y, y2 * w1.x - y1 * w2.x) / det
            val radius = origin distTo v1
            2.0 * radius * asin(chord / (2.0 * radius))
        }
    }

    private fun internalCurvature(t: Double): Double {
        val deriv = derivFromT(t)
        val derivNorm = deriv.norm()
        val secondDeriv = secondDerivFromT(t)
        return abs(secondDeriv.x * deriv.y - deriv.x * secondDeriv.y) / (derivNorm * derivNorm * derivNorm)
    }

    private fun length(tLo: Double,
                       tHi: Double,
                       vLo: Vector2d = getPointfromT(tLo).vec(),
                       vHi: Vector2d = getPointfromT(tHi).vec(),
                       depth: Int = 0): Double {
        var runningLength = 0.0
        if (depth >= 10) {
            return (vHi - vLo).norm()
        }

        val tMid = 0.5 * (tLo + tHi)
        val vMid = getPointfromT(tMid).vec()

        val deltaK = abs(internalCurvature(tLo) - internalCurvature(tHi))
        val segmentLength = approxLength(vLo, vMid, vHi)

        if (deltaK > 0.1 || segmentLength > 0.25) {
            runningLength += length(tLo, tMid, vLo, vMid, depth + 1)
            runningLength += length(tMid, tHi, vMid, vHi, depth + 1)
        } else {
            runningLength += segmentLength
        }

        return runningLength
    }


}

