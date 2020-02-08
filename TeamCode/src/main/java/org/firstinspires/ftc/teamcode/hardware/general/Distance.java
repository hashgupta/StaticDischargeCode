package org.firstinspires.ftc.teamcode.hardware.general;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.type.Device;
import org.firstinspires.ftc.teamcode.hardware.type.Input;

// rev distance sensor
public class Distance extends Device<DistanceSensor> implements Input<Double> {

	// offset from measuring distance
	private final double offset;

	// initialize sensor
	public Distance(String name, double offset, HardwareMap map) {super(map.get(DistanceSensor.class, name)); this.offset = offset;}

	// initialize sensor with default offset
	public Distance(String name, HardwareMap map) {this(name, 0, map);}

	// sense distance
	@Override public Double measure() {return device.getDistance(DistanceUnit.INCH) - offset;}
}