package org.firstinspires.ftc.teamcode.tests


import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.BuildConfig
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.firstinspires.ftc.teamcode.localizers.MockedLocalizer
import org.firstinspires.ftc.teamcode.purePursuit.FastPurePursuit
import org.junit.jupiter.api.Disabled
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import kotlin.math.abs
import kotlin.system.measureNanoTime
import kotlin.system.measureTimeMillis

@Disabled
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class PurePursuitTest {
    @Test
    fun testPurePursuitSpeeds() {
        val translationalTol = 1.0 //inches
        val angularTol = Math.toRadians(0.5) // one degree angular tolerance
        val kStatic = 0.1

        val translationalCoeffs = PIDCoefficients(0.1)
        val headingCoeffs = PIDCoefficients(0.9)

        val axialController = PIDFController(translationalCoeffs)
        val lateralController = PIDFController(translationalCoeffs, kStatic = kStatic)
        val headingController = PIDFController(headingCoeffs, kStatic = kStatic)
        axialController.update(0.0)
        lateralController.update(0.0)
        headingController.update(0.0)


        val poseError = Pose2d(10.0, 0.0, Math.toRadians(0.0))


        axialController.targetPosition = poseError.x
        lateralController.targetPosition = poseError.y
        headingController.targetPosition = poseError.heading


        // note: feedforward is processed at the wheel level
        var axialCorrection = axialController.update(0.0)
        var lateralCorrection = lateralController.update(0.0)
        var headingCorrection = headingController.update(0.0)

        println(headingCorrection)


        if (abs(poseError.x) < translationalTol) {
            axialCorrection = 0.0
        }
        if (abs(poseError.y) < translationalTol) {
            lateralCorrection = 0.0
        }
        if (abs(poseError.heading) < angularTol) {
            headingCorrection = 0.0
        }


        println(Pose2d(
                axialCorrection,
                lateralCorrection,
                headingCorrection
        ))
        val velocity = Pose2d(
                axialCorrection,
                lateralCorrection,
                headingCorrection
        )

        val vert = -velocity.x
        val hori = -velocity.y

        val turn = -velocity.heading
        val square = DriveTrain.Vector(hori, vert, turn)
        println(square.hori)
        println(square.vert)
        println(square.turn)
    }

    @Test
    fun testPursuitPaths() {

        val localizer = MockedLocalizer()
        val pursuiter = FastPurePursuit(localizer)

        val pose = Pose2d(1.0, 1.0, Math.toRadians(175.0))

        pursuiter.startAt(pose)
//        pursuiter.spline(end = Pose2d(10.0, 10.0, 0.0), endTanAngle = Math.toRadians(-135.0))
//        pursuiter.move(6.0, 7.8, 0.0)
        pursuiter.turnTo(3.141592)


        println(pursuiter.waypoints)
        pursuiter.testStep()
    //

//        for (i in 0..100) {
//            val point = pursuiter.waypoints[0].getPointfromT(i / 100.0)
//            println(point.x.toString() + "," + point.y.toString())
//        }


    }

    @Test
    fun testPurePursuitMath() {
//        val localizer = MockedLocalizer()
//        localizer.poseEstimate = Pose2d(15.0, 13.0, Math.toRadians(180.0))
//        val pursuiter = FastPurePursuit(localizer)
//
//        pursuiter.relative(10.0, 40.0, 0.0)
//
//        val target = Pose2d(-25.0, 23.0, Math.toRadians(180.0))
//        if (BuildConfig.DEBUG && !pursuiter.waypoints.last().end.epsilonEquals(target)) {
//            println(target - pursuiter.waypoints.last().end)
//            error("Assertion failed")
//        }

    }

    @Test
    fun testPurePursuitActions() {
        val localizer = MockedLocalizer()
        localizer.poseEstimate = Pose2d(15.0, 13.0, Math.toRadians(180.0))
        val pursuiter = FastPurePursuit(localizer)

        pursuiter.relative(10.0, 40.0, 0.0)

        pursuiter.action { println("action #1") }
        pursuiter.action { println("action #2") }

        if (BuildConfig.DEBUG && pursuiter.actions.size != 1) {
            println(pursuiter.actions)
            error("Assertion failed")
        }
    }

    fun speedTest() {
//        var total = 0.0
//
//        val time = measureNanoTime {
//            for (i in 1..1000) {
//                pursuiter.testStep()
//            }
//        }
//        total += time
//
//        print("Microseconds: "+total/(1000 * 1000))
    }
}