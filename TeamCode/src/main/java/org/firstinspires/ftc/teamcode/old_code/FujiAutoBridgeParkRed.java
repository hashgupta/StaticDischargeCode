package org.firstinspires.ftc.teamcode.old_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="FujiAutoBridgeParkRed", group="PatentPending")
@Disabled
public class FujiAutoBridgeParkRed extends FujiAutoBridgeParkBlue {
    @Override
    public void runOpMode() {
        setReverse();
        super.runOpMode();
    }
}