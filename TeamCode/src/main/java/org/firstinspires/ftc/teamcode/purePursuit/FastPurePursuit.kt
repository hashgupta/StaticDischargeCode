package org.firstinspires.ftc.teamcode.purePursuit

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.TankKinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.util.Angle
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.firstinspires.ftc.teamcode.opmodeTests.ROBOT_RADIUS
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin


@Config
class FastPurePursuit(val localizer: Localizer) {
    val waypoints: MutableList<Path> = mutableListOf()
    val actions: MutableList<Pair<Int, () -> Unit>> = mutableListOf()
    var index = 0
    var start:Pose2d

    val lookAhead = 8 //Look Ahead Distance, 5 is arbitrary, depends on application and needs tuning, inches

    private val translationalTol = 0.75 //inches
    private val angularTol = Math.toRadians(0.75) // one degree angular tolerance
    private val kStatic = 0.1
    var runSpeed = 0.7

    val translationalCoeffs: PIDCoefficients = PIDCoefficients(0.30)
    val headingCoeffs: PIDCoefficients = PIDCoefficients(1.00)

    private val axialController = PIDFController(translationalCoeffs)
    private val lateralController = PIDFController(translationalCoeffs, kStatic=kStatic)
    private val headingController = PIDFController(headingCoeffs, kStatic=kStatic)


    init {
        axialController.update(0.0)
        lateralController.update(0.0)
        headingController.update(0.0)

        start = localizer.poseEstimate

    }

    // follow until path is complete
    fun FollowSync(drivetrain: DriveTrain, mecanum:Boolean = true, telemetry: Telemetry) {
        runAction(0)

        var done = false

        while (!done && !Thread.currentThread().isInterrupted) {
            telemetry.addLine("running")
            telemetry.addLine(localizer.poseEstimate.toString())
            telemetry.addLine(waypoints[index].end.toString())
            telemetry.addLine(Kinematics.calculatePoseError(waypoints[index].end, localizer.poseEstimate).toString())
            telemetry.update()

            val packet = TelemetryPacket()

            val fieldOverlay: Canvas = packet.fieldOverlay()
            fieldOverlay.setStroke("#3F51B5")
            drawRobot(fieldOverlay, localizer.poseEstimate)

            fieldOverlay.setStrokeWidth(1)
            fieldOverlay.setStroke("#4CAF50")
            drawSampledPath(fieldOverlay, waypoints[index])

            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            done = followStep(drivetrain, mecanum)
        }
        this.waypoints.clear()
        index = 0
    }

    // returns whether the path is done
    fun followStep(drivetrain: DriveTrain, mecanum:Boolean = true):Boolean {
        localizer.update()

        val path = waypoints[index]

        val currentPos = localizer.poseEstimate

        val poseError = Kinematics.calculatePoseError(waypoints[index].end, currentPos)

        if (abs(poseError.x) < translationalTol && abs(poseError.y) < translationalTol &&
                abs(poseError.heading) < angularTol) {
            // go to next waypoint
            drivetrain.startFromRRPower(Pose2d(0.0,0.0,0.0), 0.0)
            runAction(index+1)

            return if (index == waypoints.size-1) {
                true
            } else {
                index += 1
                false
            }
        }

        val target : Pose2d
        val candidateGoal = path.findClosestT(currentPos) + lookAhead/path.length


        target = if (candidateGoal > 1.0 && (actions.find { it.first == index+1 } == null) && index < waypoints.size-1) {
            val excessLength = (path.findClosestT(currentPos) + (lookAhead / path.length) - 1.0) * path.length

            if (excessLength > lookAhead/4.0) {
                index += 1
                return false
            }

            waypoints[index+1].getPointfromT(limit(excessLength / waypoints[index+1].length, 0.0, 1.0))
        } else {
            path.getPointfromT(limit(candidateGoal, 0.0, 1.0))
        }


        if (mecanum) {
//            val wheelVel = getVelocityFromTarget(target = target, currentPos = currentPos)
//            drivetrain.start(DriveTrain.Square(wheelVel[3], wheelVel[2], wheelVel[0], wheelVel[1]))
            val vel = getVelocityFromTarget(target, currentPos)
            drivetrain.startFromRRPower(vel, runSpeed)
        } else {
            //TODO figure out how to make drivetrain more generic for tank and mecanum
//            val wheelVel = getWheelVelocityFromTargetTank(target, currentPos)
            print("Tank is still TODO, not fully implemented")
        }
        return false
    }
    fun followStepTestWriteup(mecanum:Boolean = true):Boolean {
        localizer.update()

        val path = waypoints[index]

        val currentPos = localizer.poseEstimate

        val poseError = Kinematics.calculatePoseError(waypoints[index].end, currentPos)


        if (abs(poseError.x) < translationalTol && abs(poseError.y) < translationalTol &&
                abs(poseError.heading) < angularTol) {
            // go to next waypoint
            runAction(index+1)

            return if (index == waypoints.size-1) {
                true
            } else {
                index += 1
                false
            }
        }

        val target : Pose2d
        val candidateGoal = path.findClosestT(currentPos) + lookAhead/path.length


        target = if (candidateGoal > 1.0 && (actions.find { it.first == index+1 } == null) && index < waypoints.size-1) {
            val excessLength = (path.findClosestT(currentPos) + (lookAhead / path.length) - 1.0) * path.length

            if (excessLength > lookAhead/4.0) {
                index += 1
                return false
            }

            waypoints[index+1].getPointfromT(limit(excessLength / waypoints[index+1].length, 0.0, 1.0))
        } else {
            path.getPointfromT(limit(candidateGoal, 0.0, 1.0))
        }


        if (mecanum) {
//            val wheelVel = getVelocityFromTarget(target = target, currentPos = currentPos)
//            drivetrain.start(DriveTrain.Square(wheelVel[3], wheelVel[2], wheelVel[0], wheelVel[1]))
            val vel = getVelocityFromTarget(target, currentPos)
            println(vel)
        } else {
            //TODO figure out how to make drivetrain more generic for tank and mecanum
//            val wheelVel = getWheelVelocityFromTargetTank(target, currentPos)
            print("Tank is still TODO, not fully implemented")
        }
        return false
    }

    fun setStartPoint(start: Pose2d) {
        localizer.poseEstimate = start
        this.start = start
    }

    fun addPoint(point: Pose2d): FastPurePursuit {
        if (waypoints.size > 0) {
            waypoints.add(LinearPath(waypoints.last().end, point))
        } else {
            waypoints.add(LinearPath(start, point))
        }
        return this
    }

    fun addRelativePoint(hori: Double, vert: Double, turn: Double):FastPurePursuit {

        val basis = if (waypoints.size > 0) {
            waypoints.last().end
        } else {
            start
        }
        val final = Pose2d(basis.x + (cos(basis.heading)*vert) + (-sin(basis.heading) * hori),
        basis.y + (sin(basis.heading)*vert) + (-cos(basis.heading)*hori), basis.heading + turn)
        addPoint(final)
        return this
    }

    fun runAction(index:Int) {

        val action = actions.find { it.first == index }?.second
        if (action != null) action()
    }


    fun addPoint(x: Double, y: Double, heading: Double): FastPurePursuit {
        addPoint(Pose2d(x, y, heading))
        return this
    }


    fun addTurn(theta: Double): FastPurePursuit {
        val last: Pose2d
        if (waypoints.size == 0) {
            last = start
        } else {
            last = waypoints.last().end
        }
        waypoints.add(TurnPath(last, Pose2d(last.vec(), Angle.norm(last.heading + theta))))
        return this
    }

    fun addTurnAbsolute(theta: Double): FastPurePursuit {
        val last: Pose2d
        if (waypoints.size == 0) {
            last = start
        } else {
            last = waypoints.last().end
        }
        waypoints.add(TurnPath(last, Pose2d(last.vec(), Angle.norm(theta))))
        return this
    }

    fun addAction(action : () -> Unit): FastPurePursuit {
        actions.add(Pair(waypoints.size, action))
        return this
    }

    fun addSpline(end:Pose2d, startTanAngle: Double, endTanAngle:Double) : FastPurePursuit {
//        val start = waypoints.last().end

        val start = if (waypoints.size > 0) {
            waypoints.last().end
        } else {
            start
        }
        waypoints.add(CubicSplinePath(start, end, startTanAngle, endTanAngle))

        return this
    }

    fun addSpline(end:Pose2d, endTanAngle:Double) : FastPurePursuit {
        val startTan = if (waypoints.last() is CubicSplinePath) {
            val path = waypoints.last() as CubicSplinePath
            path.endTangent
        } else {
            (waypoints.last().end - waypoints.last().start).vec().angle()
        }
        return addSpline(end, startTan, endTanAngle)
    }

    fun getVelocityFromTarget(target:Pose2d, currentPos:Pose2d): Pose2d {

        val error = Kinematics.calculatePoseError(target, currentPos)

        val velocity = errorToPower(error)
//        velocity = Pose2d(velocity.x + sign(velocity.x) * kStatic, velocity.y + sign(velocity.y) * kStatic, velocity.heading + sign(velocity.heading) * kStatic)

//        var wheelPow = MecanumKinematics.robotToWheelVelocities(velocity, Constants.trackwidth, Constants.wheelBase, lateralMultiplier = 1.0)
//
//        wheelPow = wheelPow.map { it + sign(it) * kStatic }
//
//        val wheelCopy = wheelPow.map {abs(it)}
//
//        if (wheelCopy.max() != null && wheelCopy.max()!! > 1) {
//            wheelPow = wheelPow.map {it/wheelCopy.max()!!}
//        }
//
//        return wheelPow
        return velocity
    }

    fun errorToPower(poseError: Pose2d): Pose2d {

        axialController.targetPosition = poseError.x
        lateralController.targetPosition = poseError.y
        headingController.targetPosition = poseError.heading


        // note: feedforward is processed at the wheel level
        var axialCorrection = axialController.update(0.0)
        var lateralCorrection = lateralController.update(0.0)
        var headingCorrection = headingController.update(0.0)


        if (abs(poseError.x) < translationalTol) {
            axialCorrection = 0.0
        }
        if (abs(poseError.y) < translationalTol) {
            lateralCorrection = 0.0
        }
        if (abs(poseError.heading) < angularTol) {
            headingCorrection = 0.0
        }

        return Pose2d(
                axialCorrection,
                lateralCorrection,
                headingCorrection
        )
    }

    fun getWheelVelocityFromTargetTank(target:Pose2d, currentPos:Pose2d): List<Double> {

        val error = Kinematics.calculatePoseError(target, currentPos)

        val velocity = TankerrorToPower(error)

        var wheelPow = TankKinematics.robotToWheelVelocities(velocity, Constants.trackwidth)

        wheelPow = wheelPow.map { it + sign(it) * kStatic }

        val wheelCopy = wheelPow.map {abs(it)}

        if (wheelCopy.max() != null && wheelCopy.max()!! > 1) {
            wheelPow = wheelPow.map {it/wheelCopy.max()!!}
        }

        return wheelPow
    }

    // TODO edit using tank pidva follower from roadrunner
    fun TankerrorToPower(poseError: Pose2d): Pose2d {
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
}


fun drawRobot(canvas: Canvas, pose: Pose2d) {
    canvas.strokeCircle(pose.x, pose.y, ROBOT_RADIUS)
    val (x, y) = pose.headingVec().times(ROBOT_RADIUS)
    val x1 = pose.x + x / 2
    val y1 = pose.y + y / 2
    val x2 = pose.x + x
    val y2 = pose.y + y
    canvas.strokeLine(x1, y1, x2, y2)
}

fun drawSampledPath(canvas: Canvas, path: Path, resolution: Double = 2.0) {
    val samples = Math.ceil(path.length / resolution).toInt()
    val xPoints = DoubleArray(samples)
    val yPoints = DoubleArray(samples)
    val dx: Double = 1.0 / samples
    for (i in 0 until samples) {
        val t = i * dx
        val pose = path.getPointfromT(t)
        val x = pose.x
        val y = pose.y
        xPoints[i] = x
        yPoints[i] = y
    }
    canvas.strokePolyline(xPoints, yPoints)
}