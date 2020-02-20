package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.FujiCode.Fuji;

public class TestGyro extends LinearOpMode {
    Fuji robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Fuji(hardwareMap, telemetry, this);

//         turns, gyro sensor
    while ( opModeIsActive() ) {
        Orientation angle  = robot.gyro.device.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("gyro reading z", angle.firstAngle);
        telemetry.addData("gyro reading y", angle.secondAngle);
        telemetry.addData("gyro reading x", angle.thirdAngle);
        telemetry.update();
        }
    }
}
