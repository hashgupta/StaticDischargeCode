import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.Controllers.Shooter
import org.firstinspires.ftc.teamcode.Controllers.shootingGoal
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.hardware.general.ServoM
import kotlin.math.PI

@TeleOp(name = "Intake Testing TeleOP", group = "Static Discharge")
class IntakeTestingTele: OpMode() {
    lateinit var intakeBottom: Motor
    lateinit var intakeTop: Motor
    var running : Boolean = false

    override fun init() {
        intakeBottom = Motor("intakeTop", 1120.0, 17.36,4.0, hardwareMap)
        intakeTop = Motor("intakeBottom", 1120.0, 17.36,4.0, hardwareMap)

    }

    override fun loop() {


        if (gamepad1.a) {
            intakeBottom.start(0.7)
            intakeTop.start(0.5)
            running = true
        } else if(gamepad1.b) {

            intakeBottom.start(-0.7)
            intakeTop.start(-0.5)
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
