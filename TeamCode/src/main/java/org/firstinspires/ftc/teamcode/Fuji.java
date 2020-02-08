package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FujiAutonomousBase;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.general.ServoM;
import org.firstinspires.ftc.teamcode.hardware.general.Motor;
import org.firstinspires.ftc.teamcode.hardware.general.Gyro;
import org.firstinspires.ftc.teamcode.hardware.general.Color;
import org.firstinspires.ftc.teamcode.hardware.general.Distance;
import org.firstinspires.ftc.teamcode.roadrun.RevSampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrun.SampleMecanumDriveBase;

// robot
public final class Fuji {

    // OpMode members
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final FujiAutonomousBase opMode;
    public final DriveTrain driveTrain;
    public final Motor lift;
    public final ServoM dropStone;
    public final ServoM pinch;
    public final ServoM hook1;
    public final ServoM hook2;
    public final Gyro gyro;
    public final Color stone;
    public final Distance distance;
    public final SampleMecanumDriveBase RoadRunnerDT;
    // robot constants
    private static final double gyroAdjust = 4;

    // initialize robot
    public Fuji(HardwareMap hardwareMap, Telemetry telemetry, FujiAutonomousBase opMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.opMode = opMode;

        Motor rf = new Motor("rf", 1120, 1, 2.95, hardwareMap);
        Motor rb = new Motor("rb", 1120, 1, 2.95, hardwareMap);
        Motor lf = new Motor("lf", 1120, 1, 2.95, hardwareMap);
        Motor lb = new Motor("lb", 1120, 1, 2.95, hardwareMap);

        driveTrain = new DriveTrain(rf, rb, lf, lb);

        RoadRunnerDT = new RevSampleMecanumDrive(hardwareMap);

        lift = new Motor("lift", 1120, 1, 2, hardwareMap);
        pinch = new ServoM("pinch", hardwareMap);
        hook1 = new ServoM("hook1", hardwareMap);
        hook2 = new ServoM("hook2", hardwareMap);
        gyro = new Gyro("imu", hardwareMap);
        dropStone = new ServoM("drop", hardwareMap);
        stone = new Color("colorFor", hardwareMap);
        distance = new Distance("distance", hardwareMap);
    }

    // turn with gyro, speed should be positive
    public void turn(double orientation) {
        while (Math.abs(headingError(orientation)) > 0.02 && opMode.opModeIsActive()) {
            telemetry.addData("Gyro Sensor", "turning");
            telemetry.addData("Angle", gyro.measure());
            telemetry.update();
            double turn = Range.clip(-headingError(orientation) * gyroAdjust, -1, 1);
            driveTrain.start(new DriveTrain.Vector(0, 0, turn).speeds());
        }
        driveTrain.start(new DriveTrain.Vector(0, 0, 0).speeds());
    }

    // drive with distance, ORIENTATION IS ONLY FOR KEEPING A HEADING, NOT FOR GOING TO A HEADING
    public void drive(double hori, double vert, double target, double orientation, boolean far) {
        while (far ? distance.measure() < target : distance.measure() > target && opMode.opModeIsActive()) {
            telemetry.addData("Distance Sensor", "driving");
            telemetry.addData("Distance", distance.measure());
            telemetry.addData("head error:", -headingError(orientation));
            telemetry.update();
            double turn = Range.clip(-headingError(orientation) * gyroAdjust, -1, 1);
            driveTrain.start(new DriveTrain.Vector(hori, vert, turn).speeds());
        }
        driveTrain.start(new DriveTrain.Vector(0, 0, 0).speeds());
    }

    // drive with encoders
    public void move(double hori, double vert) {
        telemetry.addData("Encoders", "moving");
        telemetry.addData("Horizontal", hori);
        telemetry.addData("Vertical", vert);
        telemetry.update();
        driveTrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        driveTrain.setTarget(new DriveTrain.Direction(hori, -vert, 0).speeds());
        driveTrain.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveTrain.start(new DriveTrain.Square<Double>(0.8, 0.8, 0.8, 0.8));

        // do gyro adjustment in here |
        //                            v
        while (driveTrain.isBusy() && opMode.opModeIsActive()) {}
        //turn (-headingError(angle)) * gyroAdjust)
        driveTrain.start(new DriveTrain.Square<Double>(0.0, 0.0, 0.0, 0.0));
        driveTrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // get current offset from target orientation
    public double headingError(double orientation) {
        double rawError = orientation - gyro.measure();
        if (rawError < -0.5) {rawError += 1;}
        if (rawError > 0.5) {rawError -= 1;}
        return rawError;
    }

    public void hook(double target) {
        hook1.start(target);
        hook2.start(target);
    }
}