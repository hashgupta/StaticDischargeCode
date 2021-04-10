package org.firstinspires.ftc.teamcode.cv

import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import java.util.*

class SkystonePipeline : OpenCvPipeline() {
    internal var HSVmat = Mat()
    internal var thresholdMat = Mat()
    internal var contoursOnFrameMat = Mat()
    internal var contoursList: MutableList<MatOfPoint> = ArrayList()
    internal var numContoursFound: Int = 0
    internal var avg: Double = 0.toDouble()

    private val stageToRenderToViewport = Stage.HSVMAT
    private val stages = Stage.values()

    internal enum class Stage {
        HSVMAT,
        THRESHOLD,
        CONTOURS_OVERLAYED_ON_FRAME,
        RAW_IMAGE
    }

    //    @Override
    //    public void onViewportTapped() {
    //        /*
    //         * Note that this method is invoked from the UI thread
    //         * so whatever we do here, we must do quickly.
    //         */
    //
    //        int currentStageNum = stageToRenderToViewport.ordinal();
    //
    //        int nextStageNum = currentStageNum + 1;
    //
    //        if (nextStageNum >= stages.length) {
    //            nextStageNum = 0;
    //        }
    //
    //        stageToRenderToViewport = stages[nextStageNum];
    //    }

    override fun processFrame(input: Mat): Mat {
        contoursList.clear()

        /*
         * This pipeline finds the contours of yellow blobs such as the Gold Mineral
         * from the Rover Ruckus game.
         */

        //        Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
        //        Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);
        //        Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);
        //        Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        //        numContoursFound = contoursList.size();
        //        input.copyTo(contoursOnFrameMat);
        //        Imgproc.drawContours(contoursOnFrameMat, contoursList, -1, new Scalar(0, 0, 255), 3, 8);
        //
        Imgproc.cvtColor(input, HSVmat, Imgproc.COLOR_RGB2HSV)
        Core.extractChannel(HSVmat, HSVmat, 1)
        Imgproc.rectangle(HSVmat, Point(
                input.cols() / 4.0,
                input.rows() / 4.0),
                Point(
                        (input.cols() * (3f / 4f)).toDouble(),
                        (input.rows() * (3f / 4f)).toDouble()),
                Scalar(0.0, 255.0, 0.0), 4)
        val mask = Mat(input.rows(), input.cols(), 0, Scalar(255.0))
        Imgproc.rectangle(mask, Point(
                mask.cols() / 4.0,
                mask.rows() / 4.0),
                Point(
                        (mask.cols() * (3f / 4f)).toDouble(),
                        (mask.rows() * (3f / 4f)).toDouble()),
                Scalar(0.0), Imgproc.FILLED)
        avg = Core.mean(HSVmat, mask).`val`[0]
        return HSVmat
    }

    fun average(): Double {
        return avg
    }
}