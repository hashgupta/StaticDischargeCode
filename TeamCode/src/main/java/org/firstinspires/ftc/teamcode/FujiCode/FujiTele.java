package org.firstinspires.ftc.teamcode.FujiCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FujiCode.Fuji;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;

//@TeleOp(name="FujiTele", group="PatentPending")
public final class FujiTele extends OpMode {

    // robot
    private Fuji robot;
    private boolean reverse = false;
    // field measurements
    private static final double stoneHeight = 4;
    // speeds
    private double driveSpeed = 1;
    private static final double liftUpSpeed = 1;
    private static final double liftDownSpeed = 0.5;
    private double previousX = 0;
    private double previousY = 0;
    private ElapsedTime timer = new ElapsedTime();

    private static final double maxAcceleration = 0.75;

    @Override
    public final void init() {
        //initialize and set robot behavior
        robot = new Fuji(hardwareMap, telemetry, null);
        robot.driveTrain.setZeroBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        stop();
    }

    @Override
    public final void loop() {

        // get gamepad input
        double vert = gamepad1.left_stick_y;
        double hori = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double lift = gamepad2.right_stick_y;

        boolean hookUp = gamepad1.left_bumper;
        boolean hookDown = gamepad1.right_bumper;
        boolean pinchDown = gamepad2.right_bumper;
        boolean pinchUp = gamepad2.left_bumper;

        // process input

        if (gamepad2.right_trigger > 0) {robot.dropStone.start(0.7);}
        if (gamepad2.left_trigger > 0) {robot.dropStone.start(0.5);}

        if (gamepad1.x) {reverse = true;}
        if (gamepad1.y) {reverse = false;}
        if (gamepad1.dpad_down) {driveSpeed = 0.5;}
        if (gamepad1.dpad_up) {
            driveSpeed = 1;
        }

//        double[] vector = capAcceleration(hori, vert);
//
//        hori = vector[0];
//        vert = vector[1];
//
//        timer.reset();
//        previousY = vert;
//        previousX = hori;

        if (Math.abs(vert) < 0.1) {vert = 0;}
        if (Math.abs(hori) < 0.1) {hori = 0;}

        try {
            //output values for robot movement
            robot.driveTrain.start(new DriveTrain.Vector(
            hori * driveSpeed * (reverse ? -1 : 1),
            vert * driveSpeed * (reverse ? -1 : 1),
            turn * driveSpeed).speeds());
            robot.lift.start(liftSpeed(lift));
            if (hookUp) {robot.hook(0);}
            if (hookDown) {robot.hook(0.9);}

            if (pinchDown) {
                robot.pinch.start(0.0);
            } else if (pinchUp) {
                robot.pinch.start(1.0);
            }


        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
            telemetry.addData("info", e.getStackTrace()[0].toString());
            telemetry.update();
        }
    }


    @Override
    public final void start() {}

    @Override
    public final void stop() {
        robot.driveTrain.start(new DriveTrain.Vector(0, 0, 0).speeds());
        robot.lift.start(0.0);
        robot.dropStone.start(0.5);
        robot.hook(robot.hook1.measure());
        robot.pinch.start(1.0);
    }

    private final double liftSpeed(double speed) {
        if (speed < 0) {return speed * liftUpSpeed;
        } else if (speed > 0) {return speed * liftDownSpeed;
        } else {return 0;}
    }

    private double[] capAcceleration(double horizontal, double vertical) {
        double secs = timer.seconds();

        double Xchange = (horizontal - previousX) / secs;
        double Ychange = (vertical - previousY) / secs;

        double length = Math.hypot(Xchange, Ychange);
        double angle = Math.atan2(Ychange, Xchange);
        double newlength = Math.min(length, maxAcceleration);

        horizontal = (newlength * Math.cos(angle)) + previousX;
        vertical = (newlength * Math.sin(angle)) + previousY;
        double[] output = new double[2];
        output[0] = horizontal;
        output[1] = vertical;
        return output;
    }
}