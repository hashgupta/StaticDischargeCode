package org.firstinspires.ftc.teamcode.purepursuit


import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.TankKinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.util.Angle
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.controllers.MecanumDriveTrain
import kotlin.math.abs
import kotlin.math.sign


class PurePursuit(val localizer: Localizer) {
    val waypoints: MutableList<Path> = mutableListOf()
    val actions: MutableList<Pair<Int, () -> Unit>> = mutableListOf()
    var index = 0
    var start: Pose2d

    @JvmField
    var lookAhead = 7.5 //Look Ahead Distance, 7.5 is arbitrary, depends on application and needs tuning, inches

    private val translationalTol = 0.25 //quarter inch
    private val angularTol = Math.toRadians(0.2) // quarter degree angular tolerance
    private val kStatic = 0.10 // 10.0% power regardless of distance, to overcome friction

    @JvmField
    var runSpeed = 0.925

    private val translationalCoeffs: PIDCoefficients = PIDCoefficients(0.2)
    private val headingCoeffs: PIDCoefficients = PIDCoefficients(1.2)

    private val axialController = PIDFController(translationalCoeffs)
    private val lateralController = PIDFController(translationalCoeffs)
    private val headingController = PIDFController(headingCoeffs)

    private var lastPoseError = Pose2d()


    init {
        axialController.update(0.0)
        lateralController.update(0.0)
        headingController.update(0.0)

        start = localizer.poseEstimate

    }

    // follow until path is complete
    fun follow(drivetrain: MecanumDriveTrain, mecanum: Boolean = true, telemetry: Telemetry) {
        runAction(0)

        var done = false
        var i = 0

        while (!done && !Thread.currentThread().isInterrupted) {
            if (i == 20) {
                telemetry.addLine("running")
                telemetry.addLine(localizer.poseEstimate.toString())
                telemetry.addLine(waypoints[index].end.toString())
                telemetry.update()
                i = 0
            }
            i += 1

            done = step(drivetrain, mecanum)
        }
        this.waypoints.clear()
        index = 0
    }

    // returns whether the path is done
    fun step(drivetrain: MecanumDriveTrain, mecanum: Boolean = true): Boolean {
        localizer.update()

        val path = waypoints[index]

        val currentPos = localizer.poseEstimate

        val poseError = Kinematics.calculatePoseError(waypoints[index].end, currentPos)

        if (abs(poseError.x) < translationalTol && abs(poseError.y) < translationalTol &&
                abs(poseError.heading) < angularTol) {
            // go to next waypoint
            drivetrain.startFromPose(Pose2d(0.0, 0.0, 0.0), 0.0)
            runAction(index + 1)
            lastPoseError = Pose2d()

            return if (index == waypoints.size - 1) {
                true
            } else {
                index += 1
                false
            }

        }


        val target: Pose2d
        val candidateGoal = path.findClosestT(currentPos) + lookAhead / path.length


        target = if (candidateGoal > 1.0 && (actions.find { it.first == index + 1 } == null) && index < waypoints.size - 1) {
            val excessLength = (path.findClosestT(currentPos) + (lookAhead / path.length) - 1.0) * path.length

            if (excessLength > lookAhead / 1.5) {
                index += 1
                return false
            }

            if (excessLength < 0.0) {
                path.getPointfromT(limit(candidateGoal, 0.0, 1.0))
            } else {
                waypoints[index + 1].getPointfromT(limit(excessLength / waypoints[index + 1].length, 0.0, 1.0))
            }
        } else {


            path.getPointfromT(limit(candidateGoal, 0.0, 1.0))
        }


        if (mecanum) {
            val vel = getVelocityFromTarget(target, currentPos)
            drivetrain.startFromPose(vel, runSpeed)
        } else {
            //TODO figure out how to make drivetrain more generic for tank and mecanum
//            val wheelVel = getWheelVelocityFromTargetTank(target, currentPos)
            print("Tank is still TODO, not fully implemented")
        }
        return false
    }

    fun testStep(mecanum: Boolean = true): Boolean {
        localizer.update()

        val path = waypoints[index]

        val currentPos = localizer.poseEstimate

        val poseError = Kinematics.calculatePoseError(waypoints[index].end, currentPos)


        if (abs(poseError.x) < translationalTol && abs(poseError.y) < translationalTol &&
                abs(poseError.heading) < angularTol) {
            // go to next waypoint
            runAction(index + 1)

            return if (index == waypoints.size - 1) {
                true
            } else {
                index += 1
                false
            }
        }

        val target: Pose2d
        val candidateGoal = path.findClosestT(currentPos) + lookAhead / path.length


        target = if (candidateGoal > 1.0 && (actions.find { it.first == index + 1 } == null) && index < waypoints.size - 1) {
            val excessLength = (path.findClosestT(currentPos) + (lookAhead / path.length) - 1.0) * path.length


            println(excessLength)
            println(path.findClosestT(currentPos))
            println(path.findClosestT(currentPos) + (lookAhead / path.length))

            if (excessLength > lookAhead / 1.5) {
                index += 1
                return false
            }


            if (excessLength < 0.0) {
                path.getPointfromT(limit(candidateGoal, 0.0, 1.0))
            } else {
                waypoints[index + 1].getPointfromT(limit(excessLength / waypoints[index + 1].length, 0.0, 1.0))
            }
        } else {
            println("stopping path")
            path.getPointfromT(limit(candidateGoal, 0.0, 1.0))
        }


        if (mecanum) {

            val vel = getVelocityFromTarget(target, currentPos)
            println(vel)
//            return vel
        } else {
            //TODO figure out how to make drivetrain more generic for tank and mecanum
//            val wheelVel = getWheelVelocityFromTargetTank(target, currentPos)
            print("Tank is still TODO, not fully implemented")
        }
        return false
    }

    fun startAt(start: Pose2d) {
        localizer.poseEstimate = start
        this.start = start
    }

    fun move(point: Pose2d): PurePursuit {
        if (waypoints.size > 0) {
            waypoints.add(LinearPath(waypoints.last().end, point))
        } else {
            waypoints.add(LinearPath(start, point))
        }
        return this
    }

    fun relative(right: Double, forward: Double, turn: Double): PurePursuit {

        val basis = if (waypoints.size > 0) {
            waypoints.last().end
        } else {
            start
        }

        val movementVector = Vector2d(x = forward, y = -right).rotated(basis.heading)


        val final = Pose2d(basis.vec() + movementVector, basis.heading + turn)
        move(final)
        return this
    }

    private fun runAction(index: Int) {

        val action = actions.find { it.first == index }?.second
        if (action != null) action()
    }


    fun move(x: Double, y: Double, heading: Double): PurePursuit {
        move(Pose2d(x, y, heading))
        return this
    }


    fun turn(theta: Double): PurePursuit {
        val last: Pose2d = if (waypoints.size == 0) {
            start
        } else {
            waypoints.last().end
        }
        waypoints.add(TurnPath(last, Pose2d(last.vec(), Angle.norm(last.heading + theta))))
        return this
    }

    fun turnTo(theta: Double): PurePursuit {
        val last: Pose2d = if (waypoints.size == 0) {
            start
        } else {
            waypoints.last().end
        }
        waypoints.add(TurnPath(last, Pose2d(last.vec(), Angle.norm(theta))))
        return this
    }

    fun action(action: () -> Unit): PurePursuit {

        val candidate = actions.find { it.first == waypoints.size }

        if (candidate != null) {
            // already an action at this point
            // combine the new and old actions into one action and replace in place
            val actionOld = candidate.second
            actions[actions.size - 1] = Pair(waypoints.size) { actionOld(); action() }

            return this


        }

        actions.add(Pair(waypoints.size, action))
        return this
    }

    fun spline(end: Pose2d, startTanAngle: Double, endTanAngle: Double): PurePursuit {

        val start = if (waypoints.size > 0) {
            waypoints.last().end
        } else {
            start
        }
        waypoints.add(CubicSplinePath(start, end, startTanAngle, endTanAngle))

        return this
    }

    fun getVelocityFromTarget(target: Pose2d, currentPos: Pose2d): Pose2d {

        val error = Kinematics.calculatePoseError(target, currentPos)

        var velocity = errorToPower(error)
        velocity = Pose2d(velocity.x + sign(velocity.x) * kStatic, velocity.y + sign(velocity.y) * kStatic, velocity.heading + sign(velocity.heading) * kStatic)
        return velocity
    }

    private fun errorToPower(poseError: Pose2d): Pose2d {

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

    private fun getWheelVelocityFromTargetTank(target: Pose2d, currentPos: Pose2d): List<Double> {

        val error = Kinematics.calculatePoseError(target, currentPos)

        val velocity = tankErrorToPower(error)

        var wheelPow = TankKinematics.robotToWheelVelocities(velocity, Constants.trackwidth)

        wheelPow = wheelPow.map { it + sign(it) * kStatic }

        val wheelCopy = wheelPow.map { abs(it) }



        if (wheelCopy.maxOrNull() != null && wheelCopy.maxOrNull()!! > 1) {
            wheelPow = wheelPow.map { it / wheelCopy.maxOrNull()!! }
        }

        return wheelPow
    }

    // TODO edit using tank pidva follower from roadrunner
    private fun tankErrorToPower(poseError: Pose2d): Pose2d {
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