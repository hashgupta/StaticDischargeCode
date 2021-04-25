package org.firstinspires.ftc.teamcode.matchopmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.controllers.MecanumDriveTrain
import org.firstinspires.ftc.teamcode.cv.FindRingAutoPipeline
import org.firstinspires.ftc.teamcode.cv.RingPipeline
import org.firstinspires.ftc.teamcode.robotconfigs.RobotBase
import org.openftc.easyopencv.*

abstract class GenericOpModeBase : LinearOpMode() {


    private lateinit var webcam: OpenCvCamera


    // set pipeline here, after creating it in pipelines folder
    // change these lines with new pipelines later on
    var pipeline: OpenCvPipeline = RingPipeline()


    fun setUpPipelineUltimateGoal(right: Boolean) {
        (pipeline as RingPipeline).right = right
    }


    fun initCV() {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)

        webcam.openCameraDevice()

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */


        webcam.setPipeline(pipeline)
    }

    fun initCVNoWebcam() {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId)

        webcam.openCameraDevice()

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        webcam.setPipeline(pipeline)
    }

    fun startCV() {
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
        webcam.resumeViewport()
    }

    fun stopCV() {
        webcam.stopStreaming()
        webcam.pauseViewport()
    }

    companion object {

        // field constants.
        const val TILE_LENGTH = Constants.tile_length
    }

    fun autoFindRings(gamepad: Gamepad, robot: RobotBase) {
        pipeline = FindRingAutoPipeline()

        webcam.openCameraDevice()
        webcam.setPipeline(pipeline)
        startCV()
        while (!gamepad.b && opModeIsActive()) {
            robot.localizer.update()
            val rings = (pipeline as FindRingAutoPipeline).rings()

            when (rings.size) {
                0 -> {

                    robot.driveTrain.start(MecanumDriveTrain.Vector(0.0, 0.0, 0.5).speeds())
                }
                1 -> {

                    robot.driveTrain.start(MecanumDriveTrain.Vector(0.0, 0.75, rings[0].turnControl).speeds())
                }
                else -> {

                    val sortedRings = rings.sortedWith(compareBy { it.distance })
                    val closestRing = sortedRings[0]

                    robot.driveTrain.start(MecanumDriveTrain.Vector(0.0, 0.75, closestRing.turnControl * 2).speeds())
                }
            }
        }

        stopCV()

    }


}