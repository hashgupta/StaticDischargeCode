package org.firstinspires.ftc.teamcode.hardware.general;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.hardware.type.Device;
import org.firstinspires.ftc.teamcode.hardware.type.Input;

// rev expansion hub used as a gyro sensor
public class Gyro extends Device<BNO055IMU> implements Input<Double> {

	private String name;
	// initialize sensor
	public Gyro(String name, HardwareMap map) {
		super(map.get(BNO055IMU.class, name));
		this.name = name;
		BNO055IMU.Parameters params = new BNO055IMU.Parameters();
		params.mode = BNO055IMU.SensorMode.IMU;
		params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		params.loggingEnabled = false;
		device.initialize(params);
		while (!device.isGyroCalibrated()) {}
	}

	// sense angle
	@Override public Double measure() {
		Orientation angles = device.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
		return checkRange((angles.firstAngle + 180) / 360, 0, 1, name);
	}
}