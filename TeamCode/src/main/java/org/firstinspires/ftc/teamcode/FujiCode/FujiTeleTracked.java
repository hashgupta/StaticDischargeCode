package org.firstinspires.ftc.teamcode.FujiCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Controllers.DriveTrain;


public class FujiTeleTracked extends OpMode {
    private Fuji robot;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    @Override
    public void init() {
        robot = new Fuji(hardwareMap, telemetry, null);
        robot.driveTrain.setZeroBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        stop();
    }

    @Override
    public void loop() {
        Pose2d baseVel = new Pose2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );

        Pose2d vel;
        if (Math.abs(baseVel.getX()) + Math.abs(baseVel.getY()) + Math.abs(baseVel.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(baseVel.getX())
                    + VY_WEIGHT * Math.abs(baseVel.getY())
                    + OMEGA_WEIGHT * Math.abs(baseVel.getHeading());
            vel = new Pose2d(
                    VX_WEIGHT * baseVel.getX(),
                    VY_WEIGHT * baseVel.getY(),
                    OMEGA_WEIGHT * baseVel.getHeading()
            ).div(denom);
        } else {
            vel = baseVel;
        }

//        vel = Kinematics.fieldToRobotVelocity(poseEstimate, vel); // use field centric driving when you choose too, just uncomment


        robot.RoadRunnerDT.setDrivePower(vel);

        Pose2d poseEstimate = robot.RoadRunnerDT.getPoseEstimate();

        robot.RoadRunnerDT.update();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
    }

    @Override
    public final void stop() {
        robot.driveTrain.start(new DriveTrain.Vector(0, 0, 0).speeds());
    }
}
