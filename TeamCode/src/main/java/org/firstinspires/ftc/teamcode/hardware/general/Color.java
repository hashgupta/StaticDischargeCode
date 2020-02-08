package org.firstinspires.ftc.teamcode.hardware.general;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.type.Device;
import org.firstinspires.ftc.teamcode.hardware.type.Input;

// rev color sensor
public class Color extends Device<ColorSensor> implements Input<Color.HSV> {

	// initialize sensor
	public Color(String name, HardwareMap map) {super(map.get(ColorSensor.class, name));}

	// sense color
	@Override public HSV measure() {
		float[] hsv = new float[3];
		android.graphics.Color.RGBToHSV(
			255 * device.red(),
			255 * device.green(),
			255 * device.blue(),
		hsv);
		return new HSV((double)hsv[0] / 360, (double)hsv[1]/100, (double)hsv[2]/100, "hsvClass");
	}

	// color value stored in hsv
	public static class HSV {

		// hsv values
		private final double h;
		private final double s;
		private final double v;

		// initialize color
		public HSV(double h, double s, double v, String name) {
			this.h = checkRange(h, 0, 1, "hue");
			this.s = checkRange(s, 0, 1, "saturation");
			this.v = checkRange(v, 0, 1, "value");
		}

		// get values
		public final double h() {return h;}
		public final double s() {return s;}
		public final double v() {return v;}
	}
}