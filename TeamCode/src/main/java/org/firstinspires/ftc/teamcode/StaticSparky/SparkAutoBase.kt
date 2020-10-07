package org.firstinspires.ftc.teamcode.StaticSparky

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.pipelines.RingPipeline
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

abstract class SparkAutoBase : LinearOpMode() {

    // robot
    lateinit var robot: SparkyRobot

    val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
    internal val webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)
    // OR...  Do Not Activate the Camera Monitor View
    //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

    internal var pipeline = RingPipeline() // set pipeline here, after creating it in pipelines folder

    enum class Side{
        Right,
        Left
    }

    fun initCV(side:Side) {

        webcam.openCameraDevice()

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        pipeline.right = (side == Side.Right)
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
        const val TILE_LENGTH = 23.0
    }
}