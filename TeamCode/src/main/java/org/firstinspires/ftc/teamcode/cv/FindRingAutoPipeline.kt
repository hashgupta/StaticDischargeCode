package org.firstinspires.ftc.teamcode.cv

import android.util.Log
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline


class FindRingAutoPipeline : OpenCvPipeline() {

    internal var yCbCrChan2Mat = Mat()
    var Cb = Mat()
    var thres = Mat()
    var hierarchy = Mat()
    val cameraWidth = 320.0


    /*
         * Some color constants
         */
    val GREEN = Scalar(0.0, 255.0, 0.0)
    val FocalLength = 360.0
    val ringWidth = 5.0
    val numerator = FocalLength * ringWidth

    val listofRings = mutableListOf<Ring>()

    //distance = focal length * ring width / pixel width


    /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */


    fun inputToCb(input: Mat) {
        Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb)
        Core.extractChannel(yCbCrChan2Mat, Cb, 2)
    }

    override fun init(firstFrame: Mat) {
        /*
                 * We need to call this in order to make sure the 'Cb'
                 * object is initialized, so that the submats we make
                 * will still be linked to it on subsequent frames. (If
                 * the object were to only be initialized in processFrame,
                 * then the submats would become delinked because the backing
                 * buffer would be re-allocated the first time a real frame
                 * was crunched)
                 */
        inputToCb(firstFrame)

        /*
                 * Submats are a persistent reference to a region of the parent
                 * buffer. Any changes to the child affect the parent, and the
                 * reverse also holds true.
                 */
    }

    override fun processFrame(input: Mat): Mat {

        //
        listofRings.clear()

        inputToCb(input)
        Imgproc.threshold(Cb, thres, 90.0, 255.0, 1)
        val contours = mutableListOf<MatOfPoint>()

        Imgproc.findContours(thres, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)

        val contoursPoly = mutableListOf<MatOfPoint2f>()
        val boundRect = mutableListOf<Rect>()

        for (i in 0 until contours.size) {
            contoursPoly.add(MatOfPoint2f())
            Imgproc.approxPolyDP(MatOfPoint2f(*contours[i].toArray()), contoursPoly[i], 3.0, true)
            boundRect.add(Imgproc.boundingRect(MatOfPoint(*contoursPoly[i].toArray())))
        }
//
        for (i in 0 until boundRect.size) {

            // draw green bounding rectangles on mat


            if (boundRect[i].height > 5) {
                Imgproc.rectangle(input, boundRect[i], GREEN)
                val centerx = boundRect[i].x + boundRect[i].width / 2

                val turnControl = (centerx / cameraWidth) - 0.7


                listofRings.add(Ring(numerator / boundRect[i].width, turnControl))
            }
        }
        return input

    }

    fun rings(): List<Ring> {
        return listofRings.toMutableList()
    }
}

data class Ring(val distance: Double, val turnControl: Double)