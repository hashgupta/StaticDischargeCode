package org.firstinspires.ftc.teamcode.hardware.type;

// rev component
public class Device<T> {

	// component object
    public final T device;

	// initialize component
	public Device(T device) {this.device = device;}

	// check if double is in specified range
	public static final double checkRange(double value, double min, double max, String varname) {
		if (value < min || value > max) {throw new IllegalArgumentException("double "+varname+" out of range: " + value + " min: " + min + " max: " + max);}
		return value;
	}
}