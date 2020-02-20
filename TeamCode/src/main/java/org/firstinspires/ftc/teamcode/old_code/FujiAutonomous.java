package org.firstinspires.ftc.teamcode.old_code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.FujiCode.Fuji;

public abstract class FujiAutonomous extends LinearOpMode {

    // robot
    public Fuji robot;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    // field constants.
    public static final double STONE_LENGTH = 8;
    public static final double ROBOT_EDGE_LENGTH = 18;
    public static final double STONE_BRIDGE_DISTANCE_INCH = 23.3;
    public static final double FOUNDATION_BRIDGE_DISTANCE_INCH = 34;
    public static final double BRIDGE_WALL_DISTANCE_INCH = 47;
    public static final double TILE_LENGTH = 23;
    public static final double FOUNDATION_LENGTH_INCH = 34.5;

    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;

    final String VUFORIA_KEY =
            "AfIg72b/////AAABmbgDBWvRcUvHqKNBuomofGtMqN5uaMDRcJL0KVjYweqzwaPWeSEmX9Zu2rXvDQ975QDYs6ofxGz+uEIVl0JDMl+7LLlVuFs9BixIxxKHiWFYC/ZCeQAwfxvETg+0FDiuCalGTEYDi0U5elH9eFUkHeYIj40aLS/8sb6P27TOqkYnlRS70LJILjz/jSuzy+hP63evGwyoIsHtkIp9Fl7SHaTH8zrMu1EXmTyF4nVHuZB4+bxG6AZ+jpbp8i54N2UESIvAcyrbl/JgDjavPkCeVzDzrXHzaKfmg80l9EiCjEG9XmbZEa134BZebPUFbniyCXWlmAe2Q7fJJqm/9Y8CLsm4BAStgkeJPCm5QbyxjYzz";


    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();
        tfodParameters.minimumConfidence = 0.7;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}