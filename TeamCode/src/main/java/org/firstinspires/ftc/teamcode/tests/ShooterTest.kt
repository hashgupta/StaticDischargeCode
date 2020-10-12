package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.hardware.general.Motor


class ShooterTest : OpMode(){

    private lateinit var flywheel: Motor

    override fun init(){
        flywheel = Motor("flywheel", 1.0, 1.0, hardwareMap)
    }

    override fun loop() {
        flywheel.start(gamepad1.left_stick_y.toDouble())
    }

    override fun stop() {
        flywheel.start(0.0)
    }

}

