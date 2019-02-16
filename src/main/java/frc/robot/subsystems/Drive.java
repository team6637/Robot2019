package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveManual;

public class Drive extends Subsystem {

	public boolean tunable = false;

	public WPI_TalonSRX leftMaster, leftSlave1, leftSlave2, 
	rightMaster, rightSlave1, rightSlave2;

	public DifferentialDrive drive;

	public static double turnMultiplier;

	public Drive(boolean tunable) {
		this.tunable = tunable;

		leftMaster = new WPI_TalonSRX(RobotMap.leftMasterPort);
		leftSlave1 = new WPI_TalonSRX(RobotMap.leftSlave1Port);
		leftSlave2 = new WPI_TalonSRX(RobotMap.leftSlave2Port);
		rightMaster = new WPI_TalonSRX(RobotMap.rightMasterPort);
		rightSlave1 = new WPI_TalonSRX(RobotMap.rightSlave1Port);
		rightSlave2 = new WPI_TalonSRX(RobotMap.rightSlave2Port);

		drive = new DifferentialDrive(leftMaster, rightMaster);

		// RESET TALONS
		leftMaster.configFactoryDefault();
		leftSlave1.configFactoryDefault();
		leftSlave2.configFactoryDefault();
		rightMaster.configFactoryDefault();
		rightSlave1.configFactoryDefault();
		rightSlave2.configFactoryDefault();

		// LEFT MASTER
		leftMaster.configNeutralDeadband(RobotMap.driveNeutralDeadband, RobotMap.timeoutMs);
		leftMaster.setInverted(false);
		leftSlave1.setInverted(false);
		leftSlave2.setInverted(false);
		leftMaster.setSensorPhase(false);
		leftMaster.setNeutralMode(NeutralMode.Brake);
		
		// FOLLOW
		leftSlave1.follow(leftMaster);
		leftSlave2.follow(leftMaster);
		
		// RIGHT MASTER
		rightMaster.configNeutralDeadband(RobotMap.driveNeutralDeadband, RobotMap.timeoutMs);
		rightMaster.setInverted(false);
		rightSlave1.setInverted(false);
		rightSlave2.setInverted(false);
		rightMaster.setSensorPhase(false);	
		rightMaster.setNeutralMode(NeutralMode.Brake);

		// FOLLOW
		rightSlave1.follow(rightMaster);
		rightSlave2.follow(rightMaster);
	
		turnMultiplier = .4;
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