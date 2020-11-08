//package org.firstinspires.ftc.teamcode.FujiCode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.pipelines.SkystonePipeline;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//public abstract class FujiAutonomousBase extends LinearOpMode {
//
//    // robot
//    public Fuji robot;
//
//    // field constants.
//    public static final double TILE_LENGTH = 23;
//
//    OpenCvCamera webcam;
//    SkystonePipeline pipeline = new SkystonePipeline(); // set pipeline here, after creating it in pipelines folder
//
//
//    public void initCV() {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        // OR...  Do Not Activate the Camera Monitor View
//        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
//
//        webcam.openCameraDevice();
//
//        /*
//         * Specify the image processing pipeline we wish to invoke upon receipt
//         * of a frame from the camera. Note that switching pipelines on-the-fly
//         * (while a streaming session is in flight) *IS* supported.
//         */
//        webcam.setPipeline(pipeline);
//    }
//
//    public void startCV() {
//        if (webcam != null) {
//            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            webcam.resumeViewport();
//        }
//    }
//
//    public void stopCV() {
//        if (webcam != null) {
//            webcam.stopStreaming();
//            webcam.pauseViewport();
//        }
//    }
//}
