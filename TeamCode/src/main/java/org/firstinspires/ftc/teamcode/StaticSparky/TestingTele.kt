package org.firstinspires.ftc.teamcode.StaticSparky

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.hardware.general.ServoM

@TeleOp(name = "Testing TeleOP", group = "Static Discharge")
class TestingTele: OpMode() {
    lateinit var flywheel: Motor
    lateinit var arm: ServoM
    lateinit var claw: ServoM
    override fun init() {
        flywheel = Motor("flywheel", 1120.0, 17.36,4.0, hardwareMap)
        arm = ServoM("arm", hardwareMap)
        claw = ServoM("claw", hardwareMap)
    }

    override fun loop() {
//        flywheel.start(gamepad1.left_stick_y.toDouble()*0.70)

        if (gamepad1.a) {
            arm.start(0.5)
        } else if(gamepad1.b) {
            arm.start(0.0)
        }
//        if (gamepad1.x) {
//            claw.start(0.5)
//        } else if(gamepad1.y) {
//            claw.start(0.0)
//        }
        if (gamepad1.x) {
            flywheel.setSpeed(4.5*40*3.2)
            telemetry.addData("Velocity rad/s", 5.3*40/(flywheel.r*17.36))
        }
    }

}