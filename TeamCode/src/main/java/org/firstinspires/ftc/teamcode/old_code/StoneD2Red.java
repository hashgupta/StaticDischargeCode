package org.firstinspires.ftc.teamcode.old_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="StoneD2Red", group="PatentPending")
@Disabled
public class StoneD2Red extends StoneD2 {
    @Override
    public void runOpMode() {
        setReverse();
        super.runOpMode();
    }
}
