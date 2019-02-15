package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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

	public double turnMultiplier = 0.4;

	public Drive(boolean tunable) {
		this.tunable = tunable;

		drive = new DifferentialDrive(leftMaster, rightMaster);

		// RESET TALONS
		leftMaster.configFactoryDefault();
		leftSlave1.configFactoryDefault();
		leftSlave2.configFactoryDefault();
		rightMaster.configFactoryDefault();
		rightSlave1.configFactoryDefault();
		rightSlave2.configFactoryDefault();

		// LEFT MASTER
		leftMaster.configNeutralDeadband(RobotMap.neutralDeadband, RobotMap.timeoutMs);
		leftMaster.setInverted(false);
		leftSlave1.setInverted(false);
		leftSlave2.setInverted(false);
		leftMaster.setSensorPhase(false);
		leftMaster.setNeutralMode(NeutralMode.Brake);
		
		// FOLLOW
		leftSlave1.follow(leftMaster);
		leftSlave2.follow(leftMaster);
		
		// RIGHT MASTER
		rightMaster.configNeutralDeadband(RobotMap.neutralDeadband, RobotMap.timeoutMs);
		rightMaster.setInverted(false);
		rightSlave1.setInverted(false);
		rightSlave2.setInverted(false);
		rightMaster.setSensorPhase(false);	
		rightMaster.setNeutralMode(NeutralMode.Brake);

		// FOLLOW
		rightSlave1.follow(rightMaster);
		rightSlave2.follow(rightMaster);
		
		// SAFETY
		drive.setSafetyEnabled(false);
		leftMaster.setSafetyEnabled(false);
		leftSlave1.setSafetyEnabled(false);
		leftSlave2.setSafetyEnabled(false);
		rightMaster.setSafetyEnabled(false);
		rightSlave1.setSafetyEnabled(false);
		rightSlave2.setSafetyEnabled(false);

	}
	
	// DRIVE THE MOTORS
	public void manualDrive(double move, double turn) {
    	turn = turn * turnMultiplier;
		drive.arcadeDrive(move, turn, false);
	}

	public void stop() {
		drive.tankDrive(0, 0);		
	}
	
	@Override
	public void initDefaultCommand() {
    	setDefaultCommand(new DriveManual());
	}
}