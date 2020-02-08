package org.firstinspires.ftc.teamcode.hardware.general;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.type.Device;
import org.firstinspires.ftc.teamcode.hardware.type.Input;
import org.firstinspires.ftc.teamcode.hardware.type.Output;

// rev motor
public class Motor extends Device<DcMotorEx> implements Input<Double>, Output<Double> {

	// motor information
	private final double tpr;
	private final double gr;
	private final double c;
	private final String name;

	// initialize motor
	public Motor(String name, double tpr, double gr, double d, HardwareMap map) {
		super((DcMotorEx)map.dcMotor.get(name));
		this.name = name;
        device.setTargetPositionTolerance(20); // (ticks / cpr) * (circumference * gear ratio) is inches of error from tick tolerance
		this.tpr = tpr;
		this.gr = gr;
		c = d * Math.PI;
	}

	// initialize motor with default diameter
	public Motor(String name, double tpr, double gr, HardwareMap map) {this(name, tpr, gr, 1 / Math.PI, map);}

	// initialize motor with default gear ratio
	public Motor(String name, double tpr, HardwareMap map) {this(name, tpr, 1, map);}

	// sense position
	@Override public Double measure() {return (device.getCurrentPosition() / tpr) * gr * c;}

	// start motion
	@Override public void start(Double motion) {device.setPower(checkRange(motion, -1, 1, this.name));}

	public void setTarget(double inches) {
		device.setTargetPosition((int) ((inches * tpr) / (gr * c)) + device.getCurrentPosition());
	}

	public void setMode(DcMotor.RunMode mode) {device.setMode(mode);}

    public void setZeroBehavior(DcMotor.ZeroPowerBehavior behavior) {device.setZeroPowerBehavior(behavior);}

	public boolean isBusy() {return device.isBusy();}
}