package org.firstinspires.ftc.teamcode.purePursuit

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.TankKinematics
import com.acmerobotics.roadrunner.localization.Localizer
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import java.lang.Thread.sleep
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin


@Config
class FastPurePursuit(val localizer: Localizer, startPose:Pose2d?) {
    val waypoints: MutableList<Path> = mutableListOf()
    val actions: MutableList<Pair<Int, () -> Unit>> = mutableListOf()
    var index = 0
    var start:Pose2d

    private val lookAhead = 5.0 //Look Ahead Distance, 5 is arbitrary, depends on application and needs tuning, inches

    private val translationalTol = 1.5 //inches
    private val angularTol = Math.toRadians(0.5) // one degree angular tolerance
    private val kStatic = 0.1

    private val translationalCoeffs: PIDCoefficients = PIDCoefficients(0.1)
    private val headingCoeffs: PIDCoefficients = PIDCoefficients(0.9)

    private val axialController = PIDFController(translationalCoeffs)
    private val lateralController = PIDFController(translationalCoeffs, kStatic=kStatic)
    private val headingController = PIDFController(headingCoeffs, kStatic=kStatic)

    init {
        axialController.update(0.0)
        lateralController.update(0.0)
        headingController.update(0.0)

        start = startPose ?: localizer.poseEstimate
    }

    // follow until path is complete
    fun FollowSync(drivetrain: DriveTrain, mecanum:Boolean = true, telemetry: Telemetry) {
        telemetry.addLine("starting pure pursuit")
        telemetry.addLine(waypoints.toString())

        telemetry.update()
        runAction(0)

        var done = false

        while (!done && !Thread.currentThread().isInterrupted) {
            telemetry.addLine("running")
            telemetry.addLine(localizer.poseEstimate.toString())
            telemetry.addLine(waypoints[index].end.toString())
            telemetry.addLine(Kinematics.calculatePoseError(waypoints[index].end, localizer.poseEstimate).toString())
            telemetry.update()
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

        if (currentPos.vec() distTo waypoints[index].end.vec() < translationalTol &&
                abs(currentPos.heading - waypoints[index].end.heading) < angularTol) {
            // go to next waypoint
            drivetrain.startFromRRPower(Pose2d(0.0,0.0,0.0))
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
            drivetrain.startFromRRPower(vel)
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
        waypoints.add(TurnPath(last, Pose2d(last.vec(), last.heading + theta)))
        return this
    }

    fun addTurnAbsolute(theta: Double): FastPurePursuit {
        val last: Pose2d
        if (waypoints.size == 0) {
            last = start
        } else {
            last = waypoints.last().end
        }
        waypoints.add(TurnPath(last, Pose2d(last.vec(), theta)))
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

        var velocity = errorToPower(error)

        print(velocity)
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