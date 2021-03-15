package org.firstinspires.ftc.teamcode.staticSparky

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.firstinspires.ftc.teamcode.pipelines.FindRingAutoPipeline
import org.firstinspires.ftc.teamcode.pipelines.Ring
import org.firstinspires.ftc.teamcode.pipelines.RingPipeline
import org.firstinspires.ftc.teamcode.robotConfigs.RobotBase
import org.openftc.easyopencv.*

abstract class GenericOpModeBase : LinearOpMode() {




    lateinit var webcam: OpenCvCamera


    internal var pipeline:OpenCvPipeline = RingPipeline() // set pipeline here, after creating it in pipelines folder

    enum class Side{
        Right,
        Left
    }

    fun initCV(side:Side) {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)
        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        webcam.openCameraDevice()

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        (pipeline as RingPipeline).right = (side == Side.Right)
        webcam.setPipeline(pipeline)
    }

    fun initCVNoWebcam(side:Side) {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);


        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        webcam.openCameraDevice()

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        (pipeline as RingPipeline).right = (side == Side.Right)
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
        const val TILE_LENGTH = 24.0
    }

    fun AutoFindRings(gamepad: Gamepad, robot: RobotBase) {
        pipeline = FindRingAutoPipeline()
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)
        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        webcam.openCameraDevice()
        webcam.setPipeline(pipeline)
        startCV()
        while (!gamepad.b && opModeIsActive()) {
            robot.localizer.update()
            val rings = (pipeline as FindRingAutoPipeline).rings()

            if (rings.size == 0) {

                robot.driveTrain.start(DriveTrain.Vector(0.0,0.0,0.5).speeds())
            } else if (rings.size == 1) {

                robot.driveTrain.start(DriveTrain.Vector(0.0,0.75,rings[0].turnControl).speeds())
            } else {

                val sorted_rings = rings.sortedWith(compareBy<Ring> {it.distance })
                val closest_ring = sorted_rings[0]

                robot.driveTrain.start(DriveTrain.Vector(0.0,0.75,closest_ring.turnControl*2).speeds())
            }
        }

        stopCV()

    }


}