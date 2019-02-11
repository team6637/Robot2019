package frc.robot.util;

import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotionMagic {
	
	WPI_TalonSRX talon;
	Gains gains;
	boolean tunable;
	int acceleration, cruiseVelocity;
	double targetPosition = 0;
	
	public MotionMagic(WPI_TalonSRX talon, Gains gains, int acceleration, int cruiseVelocity, boolean tunable) {
		
		// localize arguments
		this.talon = talon;
		this.gains = gains;
		this.tunable = tunable;
		this.acceleration = acceleration;
		this.cruiseVelocity = cruiseVelocity;
		
		// setting frame period to 10
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, RobotMap.timeoutMs);
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, RobotMap.timeoutMs);
		
		setParams(RobotMap.slot0, RobotMap.pidPrimary, gains, acceleration, cruiseVelocity);
		
		zeroEncoder();
		
		if(tunable) 
			outputToSmartDashboard();
		
	}
	
	public void setParams(int slot, int secondarySlot, Gains gains, int acceleration, int cruiseVelocity) {

		talon.selectProfileSlot(slot, secondarySlot);
		talon.config_kF(slot, gains.kF, RobotMap.timeoutMs);
		talon.config_kP(slot, gains.kP, RobotMap.timeoutMs);
		talon.config_kI(slot, gains.kI, RobotMap.timeoutMs);
		talon.config_kD(slot, gains.kD, RobotMap.timeoutMs);
		talon.config_IntegralZone(slot, gains.kIZone, RobotMap.timeoutMs);
		talon.configMotionCruiseVelocity(cruiseVelocity);
		talon.configMotionAcceleration(acceleration);
	}
	
	public void zeroEncoder() {
		talon.setSelectedSensorPosition(0, RobotMap.pidPrimary, RobotMap.timeoutMs);
	}
	
	public void outputToSmartDashboard() {

		SmartDashboard.putNumber("kP", gains.kP);
		SmartDashboard.putNumber("kI", gains.kI);
		SmartDashboard.putNumber("kD", gains.kD);
		SmartDashboard.putNumber("kF", gains.kF);
		SmartDashboard.putNumber("Acceleration", acceleration);
		SmartDashboard.putNumber("Cruise Velocity", cruiseVelocity);
		SmartDashboard.putNumber("Target Position", targetPosition);
		
	}
	
	// tune from smart dashboard, use outputToSmartDashboard in constructor to setup fields in SD
	public void tuneFromSmartDashboard() {

		double sdP = SmartDashboard.getNumber("kP", gains.kP);
		double sdI = SmartDashboard.getNumber("kI", gains.kI);
		double sdD = SmartDashboard.getNumber("kD", gains.kD);
		double sdF = SmartDashboard.getNumber("kF", gains.kF);
		int sdAcceleration = (int) SmartDashboard.getNumber("Acceleration", acceleration);
		int sdCruiseVelocity = (int) SmartDashboard.getNumber("Cruise Velocity", cruiseVelocity);
		double sdTargetPosition = SmartDashboard.getNumber("Target Position", targetPosition);
		if(sdP != gains.kP || sdI != gains.kI || sdD != gains.kD || sdF != gains.kF || sdAcceleration != acceleration || sdCruiseVelocity != cruiseVelocity || sdTargetPosition != targetPosition) {
			// reset MotionMagic params
			gains.kP = sdP;
			gains.kI = sdI;
			gains.kD = sdD;
			gains.kF = sdF;
			targetPosition = sdTargetPosition;
			acceleration = sdAcceleration;
			cruiseVelocity = sdCruiseVelocity;
			setParams(RobotMap.slot0, RobotMap.pidPrimary, gains, sdAcceleration, sdCruiseVelocity);
		}
	}

	public void outputMotorDataToSmartDashboard() {
		double motorOutput = talon.getMotorOutputPercent();
		double motorVelocity = talon.getSelectedSensorVelocity(RobotMap.pidPrimary);
		SmartDashboard.putNumber("Motor Output", motorOutput);
		SmartDashboard.putNumber("Velocity", motorVelocity);		
	}
}
