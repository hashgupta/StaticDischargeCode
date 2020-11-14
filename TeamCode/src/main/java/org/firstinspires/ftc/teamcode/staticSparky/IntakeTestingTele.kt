import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.hardware.general.Motor

@TeleOp(name = "Intake Testing TeleOP", group = "Static Discharge")
class IntakeTestingTele: OpMode() {
    lateinit var intakeBottom: Motor
    lateinit var intakeTop: Motor
    var running : Boolean = false

    override fun init() {
        intakeBottom = Motor("intakeBottom", 1120.0, 17.36,4.0, hardwareMap)
        intakeTop = Motor("intakeTop", 1120.0, 17.36,4.0, hardwareMap)
        intakeTop.device.direction = DcMotorSimple.Direction.REVERSE

    }

    override fun loop() {


        if (gamepad1.a) {
            intakeBottom.start(0.75)
            intakeTop.start(1.0)
            running = true
        } else if(gamepad1.b) {

            intakeBottom.start(-0.75)
            intakeTop.start(-1.0)
            running = true
        } else {
            intakeBottom.start(0.0)
            intakeTop.start(0.0)
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
