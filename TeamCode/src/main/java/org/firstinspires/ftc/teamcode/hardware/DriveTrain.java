package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.general.Motor;
import org.firstinspires.ftc.teamcode.hardware.type.Device;
import org.firstinspires.ftc.teamcode.hardware.type.Input;
import org.firstinspires.ftc.teamcode.hardware.type.Output;

// quad-omni drive train
public class DriveTrain implements Input<DriveTrain.Square<Double>>, Output<DriveTrain.Square<Double>> {

	// motors
	private final Motor rf;
	private final Motor rb;
	private final Motor lf;
	private final Motor lb;

	// initialize drive train
	public DriveTrain(Motor rf, Motor rb, Motor lf, Motor lb) {
		this.rf = rf;
		this.rb = rb;
		this.lf = lf;
		this.lb = lb;
	}

	// sense position
	@Override public Square<Double> measure() {return new Square<Double>(rf.measure(), rb.measure(), lf.measure(), lb.measure());}

	// start motion
	@Override public void start(Square<Double> motion) {
		rf.start(motion.rf);
		rb.start(motion.rb);
		lf.start(motion.lf);
		lb.start(motion.lb);
	}

	public void setTarget(Square<Double> inches) {
		rf.setTarget(inches.rf);
		rb.setTarget(inches.rb);
		lf.setTarget(inches.lf);
		lb.setTarget(inches.lb);
	}

	public void setMode(DcMotor.RunMode mode) {
		rf.setMode(mode);
		rb.setMode(mode);
		lf.setMode(mode);
		lb.setMode(mode);
	}

	public void setZeroBehavior(DcMotor.ZeroPowerBehavior behavior) {
		rf.setZeroBehavior(behavior);
		rb.setZeroBehavior(behavior);
		lf.setZeroBehavior(behavior);
		lb.setZeroBehavior(behavior);
	}


    public boolean isBusy() {
		return rf.isBusy() || rb.isBusy() || lf.isBusy() || lb.isBusy();
	}

	// vector for x, y, and turn
	public static class Direction {

		// distances
		public final double hori;
		public final double vert;
		public final double turn;

		// initialize distances
		public Direction(double hori, double vert, double turn) {
			this.hori = hori;
			this.vert = vert;
			this.turn = turn;
		}

		// get wheel distances
		public Square<Double> speeds() {
			return new Square<Double>(
				- hori - vert - turn,
				+ hori - vert - turn,
				- hori + vert - turn,
				+ hori + vert - turn);
		}
	}

	public static class Vector extends Direction {

		// initialize speeds
		public Vector(double hori, double vert, double turn) {
			super(
				Device.checkRange(hori, -1, 1, "hori"),
				Device.checkRange(vert, -1, 1, "vert"),
				Device.checkRange(turn, -1, 1, "turn"));
			/*
			if (this.hori != 0 || this.vert != 0) {this.divisor += 2;}
			if (this.turn != 0) {this.divisor += 1;}
			if (divisor == 0) {divisor = 1;}
			*/
		}

		@Override
		public Square<Double> speeds() {
			Square<Double> supers = super.speeds();
			double divisor = Math.max(
				Math.max(
					Math.abs(supers.rf),
					Math.abs(supers.rb)
				),
				Math.max(
					Math.abs(supers.lf),
					Math.abs(supers.lb)
				)
			);
            divisor = Math.max(1.0, divisor);
			return new Square<Double>(
                    supers.rf / divisor,
                    supers.rb /divisor,
                    supers.lf / divisor,
                    supers.lb / divisor);
		}
	}

	// set of data on four objects arranged in a square
	public static class Square<T> {

		// square
		public T rf;
		public T rb;
		public T lf;
		public T lb;

		// initialize square
		public Square(T rf, T rb, T lf, T lb) {
			this.rf = rf;
			this.rb = rb;
			this.lf = lf;
			this.lb = lb;
		}
	}
}