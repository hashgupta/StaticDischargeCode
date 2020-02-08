package org.firstinspires.ftc.teamcode.old_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="FujiAutoWallParkRed", group="PatentPending")
@Disabled
public class FujiAutoWallParkRed extends FujiAutoWallParkBlue {
    @Override
    public void runOpMode() {
        setReverse();
        super.runOpMode();
    }
}