package org.firstinspires.ftc.teamcode.old_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

@Disabled
@Autonomous(name = "FujiStonesTensorflow", group = "PatentPending")
public class FujiAutonomousStonesTensorflow extends FujiAutonomous {

    @Override
    public void runOpMode() {
        double skystone;
        double current;
        current = 1; //robot always begins its plans thinking its on the first stone
//        robot = new Fuji(hardwareMap, telemetry); this is the code to init robot, but since we are testing on the phone it isnt needed

        initVuforia();
        initTfod();

        tfod.activate(); //may throw exception that tfod is null, meaning that the phones dont support tensorflow

        class RecognitionPair {
            String label;
            Float xcoordinate;

            public RecognitionPair(String label, Float xcoordinate) {
                this.label = label;
                this.xcoordinate = xcoordinate;
            }
        }


        while (!isStarted()) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            List<RecognitionPair> objects = new ArrayList<RecognitionPair>();

            // CHANGED JAVA VERSION AND API VERSION, EXPERIMENTAL CODE
            if (updatedRecognitions != null) {

                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    //telemetry updates

                    String label = recognition.getLabel();
                    Float coordinate = (recognition.getLeft() + recognition.getRight()) / 2;

                    telemetry.addData("confidence: ", recognition.getConfidence());
                    telemetry.addData(String.format("label (%d)", i), label);
                    telemetry.addData(String.format("center x coordinate (%d)", i), "%.03f",
                            coordinate);

                    //add to list
                    objects.add(new RecognitionPair(label, coordinate));


                    i += 1;
                }
                telemetry.update();

                // really terrible code just to sort on x-coordinate
                Collections.sort(objects, new Comparator<RecognitionPair>() {
                    @Override
                    public int compare(RecognitionPair r1, RecognitionPair r2) {
                        return r1.xcoordinate.compareTo(r2.xcoordinate);
                    }
                });

                ArrayList<String> labels = new ArrayList<String>();

                objects.forEach((recog) -> labels.add(recog.label));


                // SKYSTONE IS SET HERE
                if (labels.contains("Skystone")) {
                    skystone = labels.indexOf("Skystone");
                } else {
                    skystone = 3;
                }



            }
        }
        //once the opmode starts, we should know what stone is the skystone, so we can immediate race off to it
        //
    }
}
