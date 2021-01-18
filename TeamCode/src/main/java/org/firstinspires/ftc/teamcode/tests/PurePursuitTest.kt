package org.firstinspires.ftc.teamcode.tests


import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.firstinspires.ftc.teamcode.purePursuit.SimplePathPurePursuit
import org.junit.jupiter.api.Disabled
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import kotlin.math.abs

@Disabled
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class PurePursuitTest {
    @Test
    fun testPurePursuitLibrary() {
        val translationalTol = 1.0 //inches
        val angularTol = Math.toRadians(0.5) // one degree angular tolerance
        val kStatic = 0.1

        val translationalCoeffs = PIDCoefficients(0.1)
        val headingCoeffs = PIDCoefficients(0.9)

        val axialController = PIDFController(translationalCoeffs)
        val lateralController = PIDFController(translationalCoeffs, kStatic=kStatic)
        val headingController = PIDFController(headingCoeffs, kStatic=kStatic)
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
        println(square.speeds())

        println(DriveTrain.Vector(0.0, 1.0, 0.0).speeds())
    }
}