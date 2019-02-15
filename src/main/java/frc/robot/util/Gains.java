package frc.robot.util;
/**
 *  Class that organizes gains used when assigning values to slots
 */
public class Gains {
	public double kP, kI, kD, kF;
	public int kIZone;
	public double kPeakOutput;
	
	public Gains(double kP, double kI, double kD){
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}

	public Gains(double kP, double kI, double kD, double kF){
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
	}
	
	public Gains(double kP, double kI, double kD, double kF, int kIZone, double kPeakOutput){
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
		this.kIZone = kIZone;
		this.kPeakOutput = kPeakOutput;
	}	
}