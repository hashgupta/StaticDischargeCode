package org.firstinspires.ftc.teamcode.old_code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.graphics.Color;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
abstract class FujiAuto extends LinearOpMode {

    // Declare timer.
    private ElapsedTime runtime = new ElapsedTime();
    // Declare OpMode members.
    private DcMotor rfMotor;
    private DcMotor lfMotor;
    private DcMotor rbMotor;
    private DcMotor lbMotor;
    private ColorSensor sensorColor;
    private DistanceSensor sensorDistance;
    DcMotor hin1;
    DcMotor hin2;
    CRServo hook1;
    CRServo hook2;
    CRServo pin;

    // Declare constants.
    private static final double PI = 3.1415;
    private static final double ROOT_TWO = 1.4142;
    // Declare drive measurements.
    private static final double DRIVE_SPEED = 0.65;
    private static final double SENSE_DISTANCE = 1;
    private static final double DRIVE_ERROR_MARGIN = 1.2;
    private static final double TURN_ERROR_MARGIN = 1.15;
    // Declare wheel measurements.
    private static final double WHEEL_DIAMETER_INCH = 3.7;
    private static final double INCH_PER_WHEEL_REV = WHEEL_DIAMETER_INCH * PI;
    // Declare motor measurements.
    private static final double COUNT_PER_MOTOR_REV = 1120; // REV Motor Encoder.
    private static final double COUNT_PER_INCH = COUNT_PER_MOTOR_REV / INCH_PER_WHEEL_REV;
    // Declare robot measurements.
    private static final double ROBOT_DIAGONAL_INCH = 19;
    private static final double INCH_PER_ROBOT_REV = ROBOT_DIAGONAL_INCH * PI;
    // Declare non-private measurements.
    static final double STONE_LENGTH_INCH = 8.6;
    static final double ROBOT_EDGE_INCH = 18;
    static final double SKYSTONE_DISTANCE_STONES = 3;
    static final double FOUNDATION_LENGTH_INCH = 34.5;
    static final double STONE_BRIDGE_DISTANCE_INCH = 23.3;
    static final double FOUNDATION_BRIDGE_DISTANCE_INCH = 34;
    static final double BRIDGE_WALL_DISTANCE_INCH = 47;
    static final double TILE_LENGTH = 25;
    // Declare non-final variables.
    private boolean reverse = false;

    /*
    // Declare arm Measurements
    private static final double MOTOR_REV_PER_ARM_REV = 72.0/15.0;
    private static final double COUNT_PER_HEX_CORE_MOTOR_REV = 288;
    */

    abstract void startGrab();
    abstract void stopGrab();

    final void setReverse() {reverse = !reverse;}
    final boolean getReverse() {return reverse;}

    private void encoderMove(double rfInch, double lfInch, double rbInch, double lbInch) {
        // Ensure that the OpMode is still active.
        if (opModeIsActive()) {
            // Set targets.
            rfMotor.setTargetPosition((int)(rfInch * COUNT_PER_INCH + rfMotor.getCurrentPosition()));
            rbMotor.setTargetPosition((int)(rbInch * COUNT_PER_INCH + rbMotor.getCurrentPosition()));
            lfMotor.setTargetPosition((int)(lfInch * COUNT_PER_INCH + lfMotor.getCurrentPosition()));
            lbMotor.setTargetPosition((int)(lbInch * COUNT_PER_INCH + lbMotor.getCurrentPosition()));
            // Set motors to RUN_TO_POSITION mode.
            rfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Reset the timer.
            runtime.reset();
            // Start motion.
            rfMotor.setPower(DRIVE_SPEED);
            lfMotor.setPower(DRIVE_SPEED);
            rbMotor.setPower(DRIVE_SPEED);
            lbMotor.setPower(DRIVE_SPEED);
            // Keep looping while we are still active, any motors are running, and there is time left.
            telemetry.addData("Move", "started moving.");
            telemetry.update();
            while (opModeIsActive() &&
                    (rfMotor.isBusy() && lfMotor.isBusy() && rbMotor.isBusy() && lbMotor.isBusy())) {}
            telemetry.addData("Move", "done moving.");
            telemetry.update();
            // Stop motion.
            rfMotor.setPower(0);
            lfMotor.setPower(0);
            rbMotor.setPower(0);
            lbMotor.setPower(0);
            // Turn off RUN_TO_POSITION mode.
            rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    final void encoderTurn(double revolutions) {
        if (reverse) {revolutions = -revolutions;}
        encoderMove(
                -revolutions * INCH_PER_ROBOT_REV * TURN_ERROR_MARGIN,
                -revolutions * INCH_PER_ROBOT_REV * TURN_ERROR_MARGIN,
                -revolutions * INCH_PER_ROBOT_REV * TURN_ERROR_MARGIN,
                -revolutions * INCH_PER_ROBOT_REV * TURN_ERROR_MARGIN);
    }

    final void encoderDrive(double forInch, double horiInch) {
        if (reverse) {horiInch = -horiInch;}
        encoderMove(
                (+ forInch - horiInch) / ROOT_TWO * DRIVE_ERROR_MARGIN,
                (- forInch - horiInch) / ROOT_TWO * DRIVE_ERROR_MARGIN,
                (+ forInch + horiInch) / ROOT_TWO * DRIVE_ERROR_MARGIN,
                (- forInch + horiInch) / ROOT_TWO * DRIVE_ERROR_MARGIN);
    }

    final void prepSense(double distance) {encoderDrive(distance - ROBOT_EDGE_INCH - SENSE_DISTANCE, 0);}
    final void nextStone(double stoneCount) {encoderDrive(0, stoneCount * STONE_LENGTH_INCH);}

    final void endLine(double speed) {
        // Get distance. Distance sensor goes from 5cm to 25cm, roughly 1.9in to 9.8in.
        double senseD = sensorDistance.getDistance(DistanceUnit.INCH);
        telemetry.addData("Distance Sensor", senseD);
        telemetry.update();
        // Start motion.
        rfMotor.setPower(-speed / 2 * DRIVE_SPEED * (reverse ? -1 : 1));
        rbMotor.setPower(+speed / 2 * DRIVE_SPEED * (reverse ? -1 : 1));
        lfMotor.setPower(-speed / 2 * DRIVE_SPEED * (reverse ? -1 : 1));
        lbMotor.setPower(+speed / 2 * DRIVE_SPEED * (reverse ? -1 : 1));
        // Wait until at correct distance.
        while (senseD < SENSE_DISTANCE + 5 && !Double.isNaN(senseD)) {
            senseD = sensorDistance.getDistance(DistanceUnit.INCH);
            telemetry.addData("Distance Sensor", senseD);
            telemetry.update();
        }
        // Stop motion.
        rfMotor.setPower(0);
        rbMotor.setPower(0);
        lfMotor.setPower(0);
        lbMotor.setPower(0);
    }

   final void upTo(double dist) {
       // Get distance. Distance sensor goes from 5cm to 25cm, roughly 1.9in to 9.8in.
       double senseD = sensorDistance.getDistance(DistanceUnit.INCH) - 2;
       telemetry.addData("Distance Sensor", senseD);
       telemetry.update();
       // Start motion.
       encoderDrive(senseD-dist, 0);
   }

    final boolean isSkystone() {
        // Declare BlockID and color.
        boolean blockID;
        float[] hsv = {0F, 0F, 0F};
        // Get HSV value.
        telemetry.addData("Color Sensor", "sensing block.");
        Color.RGBToHSV(
                sensorColor.red() * 255,
                sensorColor.green() * 255,
                sensorColor.blue() * 255,
                hsv);
        telemetry.addData("Value", hsv[2]);
        // Check which stone is sensed.
        blockID = hsv[2] <= 50;
        telemetry.addData("Block ID", blockID);
        // Return sensed stone.
        telemetry.update();
        return blockID;
    }

    final void initMotors() {
        // Initialize OpMode members.
        rfMotor = hardwareMap.dcMotor.get("rf");
        lfMotor = hardwareMap.dcMotor.get("lf");
        rbMotor = hardwareMap.dcMotor.get("rb");
        lbMotor = hardwareMap.dcMotor.get("lb");
        hin1 = hardwareMap.dcMotor.get("hin1");
        hin2 = hardwareMap.dcMotor.get("hin2");
        hook1 = hardwareMap.crservo.get("hook1");
        hook2 = hardwareMap. crservo.get("hook2");
        pin = hardwareMap.crservo.get("pinch");
        sensorColor = hardwareMap.colorSensor.get("color");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "dist");
        // Reset encoders.
        telemetry.addData("Motors", "resetting encoders.");
        rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Motors", "encoders done resetting.");
        telemetry.update();
        pin.setPower(0);
        pin.setDirection(DcMotor.Direction.REVERSE);
        // Wait for game to start (driver presses PLAY).
        waitForStart();
    }

    /*
    final void armMove(double hinRev) {
        // Ensure that the opMode is still active.
        if (opModeIsActive()) {
            // Declare motor targets.
            // counts /288 * 1/4.8 = arm rev
            //therefore counts =  288counts/shaft * 4.8shafts/armrev * arm rev
            double hingecounts = COUNT_PER_HEX_CORE_MOTOR_REV *  MOTOR_REV_PER_ARM_REV * hinRev; //set to inches per a hypotetical full arm rotation using some calculations
            // Set targets.
            hin1.setTargetPosition((int)(-hingecounts) + rfMotor.getCurrentPosition());
            hin2.setTargetPosition((int)(hingecounts) + rbMotor.getCurrentPosition());
            // Set motors to RUN_TO_POSITION mode.
            hin1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hin2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Reset the timer.
            runtime.reset();
            // Start motion.
            hin1.setPower(DRIVE_SPEED);
            hin2.setPower(DRIVE_SPEED);
            // keep looping while we are still active and any motors are running.
            telemetry.addData("Update", "Started moving.");
            telemetry.update();
            while (opModeIsActive() &&
                    (hin1.isBusy() || hin2.isBusy())) {}
            telemetry.addData("Update", "Done moving.");
            telemetry.update();
            // Stop all motion.
            hin1.setPower(0);
            hin2.setPower(0);
            // Turn off RUN_USING_ENCODER mode.
            hin1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hin2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    */
}
