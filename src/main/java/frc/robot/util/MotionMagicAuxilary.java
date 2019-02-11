package frc.robot.util;

import frc.robot.Robot;
import frc.robot.RobotMap;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotionMagicAuxilary {
	
	WPI_TalonSRX leftTalon, rightTalon;
	PigeonIMU pidgey = Robot.driveSubsystem.gyro;
	Gains gainsDistance;
	Gains gainsTurning;
	int acceleration, cruiseVelocity;
	boolean tunable;	
	
	public MotionMagicAuxilary(Gains gainsDistance, Gains gainsTurning, int acceleration, int cruiseVelocity, boolean tunable) {
		
		// localize arguments
		this.leftTalon = Robot.driveSubsystem.leftMaster;
		this.rightTalon = Robot.driveSubsystem.rightMaster; 
		this.gainsDistance = gainsDistance;
		this.gainsTurning = gainsTurning;
		this.acceleration = acceleration;
		this.cruiseVelocity = cruiseVelocity;
		this.tunable = tunable;
		

		/** Feedback Sensor Configuration */
		
		/* Configure the left Talon's selected sensor as local QuadEncoder */
		leftTalon.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,				// Local Feedback Source
													RobotMap.pidPrimary,					// PID Slot for Source [0, 1]
													RobotMap.timeoutMs);					// Configuration Timeout

		/* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
		// first reset
		//rightTalon.configRemoteFeedbackFilter(0x00, RemoteSensorSource.Off, RobotMap.remote_0, RobotMap.timeoutMs);
		rightTalon.configRemoteFeedbackFilter(leftTalon.getDeviceID(),					// Device ID of Source
												RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
												RobotMap.remote_0,							// Source number [0, 1]
												RobotMap.timeoutMs);						// Configuration Timeout
		
		/* Configure the Pigeon IMU to the other remote slot available on the right Talon */
		// first reset
		//rightTalon.configRemoteFeedbackFilter(0x00, RemoteSensorSource.Off, RobotMap.remote_0, RobotMap.timeoutMs);
		rightTalon.configRemoteFeedbackFilter(pidgey.getDeviceID(),
												RemoteSensorSource.GadgeteerPigeon_Yaw,
												RobotMap.remote_1,	
												RobotMap.timeoutMs);
		
		/* Setup Sum signal to be used for Distance */
		rightTalon.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, RobotMap.timeoutMs);				// Feedback Device of Remote Talon
		rightTalon.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder, RobotMap.timeoutMs);	// Quadrature Encoder of current Talon
		
		/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
		rightTalon.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder, 
													RobotMap.pidPrimary,
													RobotMap.timeoutMs);
		
		/* Scale Feedback by 0.5 to half the sum of Distance */
		rightTalon.configSelectedFeedbackCoefficient(	0.5 * 3, 						// Coefficient
														RobotMap.pidPrimary,		// PID Slot of Source 
														RobotMap.timeoutMs);		// Configuration Timeout
		
		/* Configure Remote 1 [Pigeon IMU's Yaw] to be used for Auxiliary PID Index */
		rightTalon.configSelectedFeedbackSensor(	FeedbackDevice.RemoteSensor1,
													RobotMap.pidTurn,
													RobotMap.timeoutMs);
		
		/* Scale the Feedback Sensor using a coefficient */
		rightTalon.configSelectedFeedbackCoefficient(	RobotMap.turnTravelUnitsPerRotation / RobotMap.pigeonUnitsPerRotation,
														RobotMap.pidTurn,
														RobotMap.timeoutMs);
		
		// set inversion and stuff
		leftTalon.setInverted(false);
		leftTalon.setSensorPhase(false);
		Robot.driveSubsystem.leftSlave1.setInverted(false);
		Robot.driveSubsystem.leftSlave2.setInverted(false);
		
		rightTalon.setInverted(true);
		rightTalon.setSensorPhase(false);
		Robot.driveSubsystem.rightSlave1.setInverted(true);
		Robot.driveSubsystem.rightSlave2.setInverted(true);
						
		/* Set status frame periods to ensure we don't have stale data */
		rightTalon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, RobotMap.timeoutMs);
		rightTalon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, RobotMap.timeoutMs);
		rightTalon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, RobotMap.timeoutMs);
		rightTalon.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, RobotMap.timeoutMs);
		leftTalon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, RobotMap.timeoutMs);
		pidgey.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR , 5, RobotMap.timeoutMs);

		
		/* 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		rightTalon.configSetParameter(ParamEnum.ePIDLoopPeriod, closedLoopTimeMs, 0x00, 0, RobotMap.timeoutMs);
		rightTalon.configSetParameter(ParamEnum.ePIDLoopPeriod, closedLoopTimeMs, 0x00, 1, RobotMap.timeoutMs);
		
		//rightTalon.configSetParameter(ParamEnum.eRemoteSensorClosedLoopDisableNeutralOnLOS,closedLoopTimeMs, 0x00, 0x00, RobotMap.timeoutMs);

		/* configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		rightTalon.configAuxPIDPolarity(false, RobotMap.timeoutMs);
		
		/* Determine which slot affects which PID */
		rightTalon.selectProfileSlot(RobotMap.slot0, RobotMap.pidPrimary);
		rightTalon.selectProfileSlot(RobotMap.slot1, RobotMap.pidTurn);
		
		setParams(RobotMap.slot0, RobotMap.pidPrimary, gainsDistance, gainsTurning, acceleration, cruiseVelocity);
		
		zeroSensors();
		
		if(tunable) 
			outputToSmartDashboard();
		
	}
	
	public void setParams(int slot, int secondarySlot, Gains gainsDistance, Gains gainsTurning, int acceleration, int cruiseVelocity) {
				
		/* Motion Magic Configurations */
		rightTalon.configMotionAcceleration(acceleration, RobotMap.timeoutMs);
		rightTalon.configMotionCruiseVelocity(cruiseVelocity, RobotMap.timeoutMs);

		/* FPID Gains for distance servo */
		rightTalon.config_kP(RobotMap.slot0, gainsDistance.kP, RobotMap.timeoutMs);
		rightTalon.config_kI(RobotMap.slot0, gainsDistance.kI, RobotMap.timeoutMs);
		rightTalon.config_kD(RobotMap.slot0, gainsDistance.kD, RobotMap.timeoutMs);
		rightTalon.config_kF(RobotMap.slot0, gainsDistance.kF, RobotMap.timeoutMs);
		rightTalon.config_IntegralZone(RobotMap.slot0, (int)gainsDistance.kIZone, RobotMap.timeoutMs);
		rightTalon.configClosedLoopPeakOutput(RobotMap.slot0, gainsDistance.kPeakOutput, RobotMap.timeoutMs);
		rightTalon.configAllowableClosedloopError(RobotMap.slot0, 0, RobotMap.timeoutMs);

		/* FPID Gains for turn servo */
		rightTalon.config_kP(RobotMap.slot1, gainsTurning.kP, RobotMap.timeoutMs);
		rightTalon.config_kI(RobotMap.slot1, gainsTurning.kI, RobotMap.timeoutMs);
		rightTalon.config_kD(RobotMap.slot1, gainsTurning.kD, RobotMap.timeoutMs);
		rightTalon.config_kF(RobotMap.slot1, gainsTurning.kF, RobotMap.timeoutMs);
		rightTalon.config_IntegralZone(RobotMap.slot1, (int)gainsTurning.kIZone, RobotMap.timeoutMs);
		rightTalon.configClosedLoopPeakOutput(RobotMap.slot1, gainsTurning.kPeakOutput, RobotMap.timeoutMs);
		rightTalon.configAllowableClosedloopError(RobotMap.slot1, 0, RobotMap.timeoutMs);

	}
	
	public void zeroSensors() {
		leftTalon.getSensorCollection().setQuadraturePosition(0, RobotMap.timeoutMs);
		rightTalon.getSensorCollection().setQuadraturePosition(0, RobotMap.timeoutMs);
		pidgey.setYaw(0, RobotMap.timeoutMs);
		pidgey.setAccumZAngle(0, RobotMap.timeoutMs);
	}
	
	public void outputToSmartDashboard() {

		SmartDashboard.putNumber("kP", gainsDistance.kP);
		SmartDashboard.putNumber("kI", gainsDistance.kI);
		SmartDashboard.putNumber("kD", gainsDistance.kD);
		SmartDashboard.putNumber("kF", gainsDistance.kF);
		
		SmartDashboard.putNumber("kP Turning", gainsTurning.kP);
		SmartDashboard.putNumber("kI Turning", gainsTurning.kI);
		SmartDashboard.putNumber("kD Turning", gainsTurning.kD);
		SmartDashboard.putNumber("kF Turning", gainsTurning.kF);
		
		SmartDashboard.putNumber("Acceleration", acceleration);
		SmartDashboard.putNumber("Cruise Velocity", cruiseVelocity);
		
	}
	
	// tune from smart dashboard, use outputToSmartDashboard in constructor to setup fields in SD
	public void tuneFromSmartDashboard() {
		
		double sdP = SmartDashboard.getNumber("kP", gainsDistance.kP);
		double sdI = SmartDashboard.getNumber("kI", gainsDistance.kI);
		double sdD = SmartDashboard.getNumber("kD", gainsDistance.kD);
		double sdF = SmartDashboard.getNumber("kF", gainsDistance.kF);
		
		double sdPTurning = SmartDashboard.getNumber("kP Turning", gainsTurning.kP);
		double sdITurning = SmartDashboard.getNumber("kI Turning", gainsTurning.kI);
		double sdDTurning = SmartDashboard.getNumber("kD Turning", gainsTurning.kD);
		double sdFTurning = SmartDashboard.getNumber("kF Turning", gainsTurning.kF);
		
		int sdAcceleration = (int) SmartDashboard.getNumber("Acceleration", acceleration);
		int sdCruiseVelocity = (int) SmartDashboard.getNumber("Cruise Velocity", cruiseVelocity);
		
		if(sdP != gainsDistance.kP || sdI != gainsDistance.kI || sdD != gainsDistance.kD || sdF != gainsDistance.kF
			|| sdPTurning != gainsTurning.kP || sdITurning != gainsTurning.kI || sdDTurning != gainsTurning.kD || sdFTurning != gainsTurning.kF
			|| sdAcceleration != acceleration || sdCruiseVelocity != cruiseVelocity) {
			// reset MotionMagic params
			gainsDistance.kP = sdP;
			gainsDistance.kI = sdI;
			gainsDistance.kD = sdD;
			gainsDistance.kF = sdF;
			
			gainsTurning.kP = sdPTurning;
			gainsTurning.kI = sdITurning;
			gainsTurning.kD = sdDTurning;
			gainsTurning.kF = sdFTurning;
			
			acceleration = sdAcceleration;
			cruiseVelocity = sdCruiseVelocity;
			setParams(RobotMap.slot0, RobotMap.pidPrimary, gainsDistance, gainsTurning, sdAcceleration, sdCruiseVelocity);
		
			SmartDashboard.putString("Console", "MM setParams tuned from dashboard " + Math.random());
		}
	}

	public void outputMotorDataToSmartDashboard() {
		double leftMotorOutput = leftTalon.getMotorOutputPercent();
		double leftMotorVelocity = leftTalon.getSelectedSensorVelocity(RobotMap.pidPrimary);
		SmartDashboard.putNumber("Left Motor Output", leftMotorOutput);
		SmartDashboard.putNumber("Left Velocity", leftMotorVelocity);		

		double rightMotorOutput = rightTalon.getMotorOutputPercent();
		double rightMotorVelocity = rightTalon.getSelectedSensorVelocity(RobotMap.pidPrimary);
		SmartDashboard.putNumber("Right Motor Output", rightMotorOutput);
		SmartDashboard.putNumber("Right Velocity", rightMotorVelocity);
	}
}
