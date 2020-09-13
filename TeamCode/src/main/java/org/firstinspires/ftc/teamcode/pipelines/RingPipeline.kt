package org.firstinspires.ftc.teamcode.pipelines

import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline


class RingPipeline: OpenCvPipeline(){

    internal var yCbCrChan2Mat = Mat()
    internal var avg: Double = 0.toDouble()
    var region_Cb: Mat? = null
    var Cb = Mat()

    /*
         * Some color constants
         */
    val BLUE = Scalar(0.0, 0.0, 255.0)
    val GREEN = Scalar(0.0, 255.0, 0.0)

    /*
         * The core values which define the location and size of the sample regions
         */
    val REGION_TOPLEFT_ANCHOR_POINT = Point(109.0, 98.0)
    val REGION_WIDTH = 20
    val REGION_HEIGHT = 20

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
    var region_pointA = Point(
            REGION_TOPLEFT_ANCHOR_POINT.x,
            REGION_TOPLEFT_ANCHOR_POINT.y)
    var region_pointB = Point(
            REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT)





    enum class RingCount {
        A,
        B,
        C
    }

    fun inputToCb(input: Mat?) {
        Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb)
        Core.extractChannel(yCbCrChan2Mat, Cb, 2)
    }

    fun init(firstFrame: Mat?): Unit {
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
        region_Cb = Cb.submat(Rect(region_pointA, region_pointB))
}

    override fun processFrame(input: Mat): Mat {

        /*
         * This pipeline finds the contours of yellow blobs such as the Gold Mineral
         * from the Rover Ruckus game.
         */

        //

        inputToCb(input);


        avg = Core.mean(region_Cb).`val`[0]

        Imgproc.rectangle(input, region_pointA,
                region_pointB,
                BLUE, 4)


        return input
    }

    fun average(): Double {
        return avg
    }
}