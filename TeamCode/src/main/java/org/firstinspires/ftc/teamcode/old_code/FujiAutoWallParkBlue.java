package org.firstinspires.ftc.teamcode.old_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="FujiAutoWallParkBlue", group="PatentPending")
@Disabled
public class FujiAutoWallParkBlue extends FujiAuto {

    @Override
    public void runOpMode() {

        initMotors();
        telemetry.addData("Path", "started.");
        telemetry.update();

        encoderDrive(0, 30);

        telemetry.addData("Path", "complete.");
        telemetry.update();
    }

    @Override final void startGrab() {}
    @Override final void stopGrab() {}
}
