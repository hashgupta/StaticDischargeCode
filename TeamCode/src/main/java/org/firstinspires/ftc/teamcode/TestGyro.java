package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.StaticSparky.SparkyRobot;

public class TestGyro extends LinearOpMode {
    SparkyRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SparkyRobot(hardwareMap, telemetry, this);

//         turns, gyro sensor
//    while ( opModeIsActive() ) {
//        Orientation angle  = robot.gyro.getDevice().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        telemetry.addData("gyro reading z", angle.firstAngle);
//        telemetry.addData("gyro reading y", angle.secondAngle);
//        telemetry.addData("gyro reading x", angle.thirdAngle);
//        telemetry.update();
//        }
    }
}
