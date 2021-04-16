package org.firstinspires.ftc.teamcode.matchopmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import org.firstinspires.ftc.teamcode.controllers.MecanumDriveTrain.Vector;
import org.firstinspires.ftc.teamcode.robotconfigs.TestRobot;

@Disabled
@TeleOp(name = "Example TeleOp", group = "StaticDischarge")
public final class ExampleTeleJava extends OpMode {
    private TestRobot robot;
    private final double driveSpeed = 1.0;

    public void init() {
        this.robot = new TestRobot(this.hardwareMap, this.telemetry, null);

        robot.getLocalizer().setPoseEstimate(new Pose2d(0.0, 0.0, 0.0));
        robot.getDriveTrain().setZeroBehavior(ZeroPowerBehavior.FLOAT);

        this.stop();
    }

    public void loop() {

        robot.getLocalizer().update();
        double vert = (double) -gamepad1.left_stick_y;
        double hori = (double) gamepad1.left_stick_x;
        double turn = (double) gamepad1.right_stick_x;

        try {

            robot.getDriveTrain().start((new Vector(hori * this.driveSpeed, vert * this.driveSpeed, turn * this.driveSpeed)).speeds());


            telemetry.addData("Pose Estimate", robot.getLocalizer().getPoseEstimate());
            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage(), new Object[0]);
            telemetry.addData("info", e.getStackTrace()[0].toString(), new Object[0]);
            telemetry.update();
        }

    }

    public void start() {
    }

    public void stop() {

        robot.getDriveTrain().start((new Vector(0.0, 0.0, 0.0)).speeds());
    }
}

