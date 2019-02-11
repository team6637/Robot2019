package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.DriveManual;

public class Drive extends Subsystem {

	public boolean tunable = false;

	public WPI_TalonSRX leftMaster = new WPI_TalonSRX(RobotMap.leftMasterPort);
	public WPI_TalonSRX leftSlave1 = new WPI_TalonSRX(RobotMap.leftSlave1Port);
	public WPI_TalonSRX leftSlave2 = new WPI_TalonSRX(RobotMap.leftSlave2Port);
	public WPI_TalonSRX rightMaster = new WPI_TalonSRX(RobotMap.rightMasterPort);
	public WPI_TalonSRX rightSlave1 = new WPI_TalonSRX(RobotMap.rightSlave1Port);
	public WPI_TalonSRX rightSlave2 = new WPI_TalonSRX(RobotMap.rightSlave2Port);

	public DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

	public PigeonIMU gyro;

	public double turnMultiplier = 0.3;

	public Drive(boolean tunable) {
		
		this.tunable = tunable;
		
		if(tunable) {
			SmartDashboard.putNumber("Drive Turn Multiplier", turnMultiplier);
		}

		drive = new DifferentialDrive(leftMaster, rightMaster);

		// LEFT MASTER
		leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, RobotMap.pidPrimary, RobotMap.timeoutMs);		
		leftMaster.configNeutralDeadband(RobotMap.neutralDeadband, RobotMap.timeoutMs);
		
		// LEFT SLAVES
		leftSlave1.follow(leftMaster);
		leftSlave2.follow(leftMaster);
		
		// RIGHT MASTER
		rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, RobotMap.pidPrimary, RobotMap.timeoutMs);
		rightMaster.configNeutralDeadband(RobotMap.neutralDeadband, RobotMap.timeoutMs);
		
		// RIGHT SLAVE
		rightSlave1.follow(rightMaster);
		rightSlave2.follow(rightMaster);
		
		gyro = new PigeonIMU(rightSlave1);
		
    	resetAngle();
    	resetEncoders();  	
	}
	
	// SETUP TALONS
	// called in auton an teleop init in Robot.java
	public void resetTalons(boolean isAuton) {
		if(isAuton) {

			setTalonDefaults(leftMaster);
			leftMaster.setInverted(false);
			leftSlave1.setInverted(false);
			leftSlave2.setInverted(false);
			leftMaster.setSensorPhase(false);
			leftMaster.setNeutralMode(NeutralMode.Brake);
	    	leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, RobotMap.timeoutMs);
			
			setTalonDefaults(rightMaster);
			rightMaster.setInverted(true);
			rightSlave1.setInverted(true);
			rightSlave2.setInverted(true);
			rightMaster.setSensorPhase(false);
			rightMaster.setNeutralMode(NeutralMode.Brake);
			rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, RobotMap.timeoutMs);
			
			
			// turn off safety mode
			// since auton uses talons, drive gets mad
			drive.setSafetyEnabled(false);
			leftMaster.setSafetyEnabled(false);
			leftSlave1.setSafetyEnabled(false);
			
			rightMaster.setSafetyEnabled(false);
			rightSlave1.setSafetyEnabled(false);
			rightSlave2.setSafetyEnabled(false);
			
		} else {
			
			setTalonDefaults(leftMaster);
			leftMaster.setInverted(false);
			leftSlave1.setInverted(false);
			leftSlave2.setInverted(false);
			leftMaster.setSensorPhase(false);
			leftMaster.setNeutralMode(NeutralMode.Brake);
			leftMaster.set(ControlMode.PercentOutput, 0);
			leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, RobotMap.timeoutMs);
			
			setTalonDefaults(rightMaster);
			rightMaster.setInverted(false);
			rightSlave1.setInverted(false);
			rightSlave2.setInverted(false);
			rightMaster.setSensorPhase(false);	
			rightMaster.setNeutralMode(NeutralMode.Brake);
			rightMaster.set(ControlMode.PercentOutput, 0);
			rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, RobotMap.timeoutMs);
		}
	}
	
	// called by resetTalons()
	public void setTalonDefaults(WPI_TalonSRX talon) {
		talon.configNominalOutputForward(0, RobotMap.timeoutMs);
		talon.configNominalOutputReverse(0, RobotMap.timeoutMs);
		talon.configPeakOutputForward(1, RobotMap.timeoutMs);
		talon.configPeakOutputReverse(-1, RobotMap.timeoutMs);
		talon.configSelectedFeedbackCoefficient( 1, RobotMap.pidPrimary, RobotMap.timeoutMs);
		talon.configSelectedFeedbackCoefficient( 1, RobotMap.pidTurn, RobotMap.timeoutMs);
		talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, RobotMap.pidPrimary, RobotMap.timeoutMs);
		talon.configForwardSoftLimitEnable(false, RobotMap.timeoutMs);
	}
	
	// DRIVE THE MOTORS
	public void manualDrive(double move, double turn) {
		if(tunable) {
			turnMultiplier = SmartDashboard.getNumber("Drive Turn Multiplier", turnMultiplier);
		}
		
    	turn = turn * turnMultiplier;
		drive.arcadeDrive(move, turn, false);

	}
	public void autonDrive(double leftPower, double rightPower) {
		
		leftMaster.set(ControlMode.PercentOutput, leftPower);
		rightMaster.set(ControlMode.PercentOutput, rightPower);
	}

	public void stop() {
		leftMaster.set(ControlMode.PercentOutput, 0);
		rightMaster.set(ControlMode.PercentOutput, 0);		
	}
	
	// GYRO  
    public double getAngle() {
    	double [] ypr = new double[3];
    	gyro.getYawPitchRoll(ypr);
    	double angle = ypr[0];
    	return -angle;
    }
    
    public void resetAngle() {
    	gyro.setYaw(0.0, 0);
    }
    
    // ENCODERS
	public void resetEncoders() {
		leftMaster.setSelectedSensorPosition(0, 0, RobotMap.timeoutMs);
		rightMaster.setSelectedSensorPosition(0, 0, RobotMap.timeoutMs);
	}
	
	public int getLeftTicks() {
		return leftMaster.getSelectedSensorPosition(0);
	}

	public int getRightTicks() {
		return rightMaster.getSelectedSensorPosition(0);
	}
	
	public int getAverageTicks() {
		return (getLeftTicks() + getRightTicks()) / 2;
	}
	
	public double ticksToInches(int ticks) {
		return ticks / RobotMap.sensorUnitsPerRotation * (RobotMap.wheelDiameter * Math.PI);
	}

	public double getLeftPositionInInches() {
		return ticksToInches(getLeftTicks());
	}
	
	public double getRightPositionInInches() {
		return ticksToInches(getRightTicks());
	}
	public double getAveragePositionInInches() {
		return ticksToInches((getLeftTicks() + getRightTicks()) / 2);
	}
	
	public double ticksFromInches(double inches) {
		double rotations = inches / RobotMap.actualWheelCircumference;
		return rotations * RobotMap.sensorUnitsPerRotation;
	}
	
	@Override
	public void initDefaultCommand() {
    	setDefaultCommand(new DriveManual());
	}
}