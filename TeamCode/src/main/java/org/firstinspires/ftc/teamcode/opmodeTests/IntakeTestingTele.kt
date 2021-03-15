package org.firstinspires.ftc.teamcode.opmodeTests

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.hardware.general.Motor

@TeleOp(name = "Intake Testing TeleOP", group = "Static Discharge")
class IntakeTestingTele: OpMode() {
    lateinit var intake: Motor
//    lateinit var intakeTop: Motor
    var running : Boolean = false

    override fun init() {
        intake = Motor("intake", 1120.0, 1.0,4.0, hardwareMap)
//        intakeTop = Motor("intakeTop", 1120.0, 17.36,4.0, hardwareMap)
//        intake.device.direction = DcMotorSimple.Direction.REVERSE

    }

    override fun loop() {


        if (gamepad1.a) {
            intake.start(1.0)

            running = true
        } else if(gamepad1.b) {

            intake.start(-1.0)
            running = true
        } else {
            intake.start(0.0)
            running = false
        }

        if (running) {
            telemetry.addData("On", true)
        } else {
            telemetry.addData("On", false)
        }

        telemetry.update()

    }

}
