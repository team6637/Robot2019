package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PID {

	Gains gains;
	boolean tunable;
	
	double target = 0;
	double lastError;
	double totalError;
	
	public PID(Gains gains, double target, boolean tunable) {
		this.gains = gains;
		this.tunable = tunable;
		lastError = 0;
		totalError = 0;
		
		setTarget(target);
		
		if(tunable) 
			outputToSmartDashboard();	
	}
	
	public void setTarget(double target) {
		this.target = target;
	}
	public void setP(double kP) {
		gains.kP = kP;
	}
	public void setI(double kI) {
		gains.kI = kI;
	}
	public void setD(double kD) {
		gains.kD = kD;
	}
	public double getCorrection(double current){
		
		//if(newTarget != this.target) {
		//	setTarget(newTarget);
		//}

		// Tune from SmartDashboard
		if(tunable)
			tuneFromSmartDashboard();
		
		double error = target - current;
		totalError = totalError + error;
		double changeInError = error - lastError;
		lastError = error;
		
		double correction = gains.kP * error + gains.kI * totalError + gains.kD * changeInError;
    	if (correction > 1) correction = 1;
    	if (correction < -1) correction = -1;
    	
    	if(tunable)
    		SmartDashboard.putNumber("Correction", correction);
    	    	
    	return correction;
	}
	
	// SMART DASHBOARD TUNING
	public void outputToSmartDashboard() {
		SmartDashboard.putNumber("kP", gains.kP);
		SmartDashboard.putNumber("kI", gains.kI);
		SmartDashboard.putNumber("kD", gains.kD);		
		SmartDashboard.putNumber("Target Position", target);
	}
	
	public void tuneFromSmartDashboard() {
		double sdP = SmartDashboard.getNumber("kP", 0);
		double sdI = SmartDashboard.getNumber("kI", 0);
		double sdD = SmartDashboard.getNumber("kD", 0);
		if(sdP != gains.kP) {
			setP(sdP);
		}
		if(sdI != gains.kI) {
			setI(sdI);
		}
		if(sdD != gains.kD) {
			setD(sdD);
		}
		double sdTarget = SmartDashboard.getNumber("Target Position", 0);
		if(sdTarget != target){
			setTarget(sdTarget);
		}	
	}
}