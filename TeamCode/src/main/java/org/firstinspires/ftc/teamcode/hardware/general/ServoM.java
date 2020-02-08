package org.firstinspires.ftc.teamcode.hardware.general;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.type.Device;
import org.firstinspires.ftc.teamcode.hardware.type.Input;
import org.firstinspires.ftc.teamcode.hardware.type.Output;

// rev servo
public class ServoM extends Device<Servo> implements Input<Double>, Output<Double> {

	private String name;
	// initialize servo
	public ServoM(String name, HardwareMap map) {super(map.servo.get(name)); this.name = name;}

	// sense position
	@Override public Double measure() {return device.getPosition();}

	// start motion
	@Override public void start(Double motion) {device.setPosition(checkRange(motion, 0, 1, this.name));}
}