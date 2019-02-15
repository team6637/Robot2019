package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.HatchEject;
import frc.robot.commands.IntakeBringIt;
import frc.robot.commands.IntakeSendIt;
import frc.robot.commands.JackFrontLower;
import frc.robot.commands.JackFrontRaise;
import frc.robot.commands.LiftSetPosition;

public class OI {
	
	// setup main joystick
	public Joystick stick = new Joystick(RobotMap.joystickPort);

	Button rocketCargo1 = new JoystickButton(stick, 11);
	Button rocketCargo2 = new JoystickButton(stick, 9);
	Button rocketCargo3 = new JoystickButton(stick, 7);
	Button rocketHatch1 = new JoystickButton(stick, 12);
	Button rocketHatch2 = new JoystickButton(stick, 10);
	Button rocketHatch3 = new JoystickButton(stick, 8);

	Button cargoIntakePosition = new JoystickButton(stick, 4);
	Button homePosition = new JoystickButton(stick, 3);

	Button intakeBringIt = new JoystickButton(stick, 2);
	Button intakeSendIt = new JoystickButton(stick, 1);

	// setup the control panel
	public Joystick controlPanel = new Joystick(1);

	Button hatchEject = new JoystickButton(controlPanel, 9);
	Button frontJackRaise = new JoystickButton(controlPanel, 11);
	Button frontJackLower = new JoystickButton(controlPanel, 12);

	
	public OI() {
		// joystick
		intakeBringIt.whileHeld(new IntakeBringIt());
		intakeSendIt.whileHeld(new IntakeSendIt());

		// setup variables to hold the preset lift positions
		int liftStartingPosition = Robot.liftSubsystem.getStartingPosition();
		int liftCargoIntakePosition = Robot.liftSubsystem.getCargoIntakePosition();
		int liftRocketCargo1Position = Robot.liftSubsystem.getRocketCargo1Position();
		int liftRocketCargo2Position = Robot.liftSubsystem.getRocketCargo2Position();
		int liftRocketCargo3Position = Robot.liftSubsystem.getRocketCargo3Position();
		int liftRocketHatch1Position = Robot.liftSubsystem.getRocketHatch1Position();
		int liftRocketHatch2Position = Robot.liftSubsystem.getRocketHatch2Position();
		int liftRocketHatch3Position = Robot.liftSubsystem.getRocketHatch3Position();

		// setup variables to hold the preset wrist positions
		int wristStartingPosition = Robot.wristSubsystem.getStartingPosition();
		int wristCargoIntakePosition = Robot.wristSubsystem.getCargoIntakePosition();
		int wristRocketCargo1Position = Robot.wristSubsystem.getRocketCargo1Position();
		int wristRocketCargo2Position = Robot.wristSubsystem.getRocketCargo2Position();
		int wristRocketCargo3Position = Robot.wristSubsystem.getRocketCargo3Position();
		int wristRocketHatch1Position = Robot.wristSubsystem.getRocketHatch1Position();
		int wristRocketHatch2Position = Robot.wristSubsystem.getRocketHatch2Position();
		int wristRocketHatch3Position = Robot.wristSubsystem.getRocketHatch3Position();

		// pass the position variables on the 6 side buttons to the LiftSetPosition Command
		rocketCargo1.whenPressed(new LiftSetPosition(liftRocketCargo1Position, wristRocketCargo1Position));
		rocketCargo2.whenPressed(new LiftSetPosition(liftRocketCargo2Position, wristRocketCargo2Position));	
		rocketCargo3.whenPressed(new LiftSetPosition(liftRocketCargo3Position, wristRocketCargo3Position));
		rocketHatch1.whenPressed(new LiftSetPosition(liftRocketHatch1Position, wristRocketHatch1Position));
		rocketHatch2.whenPressed(new LiftSetPosition(liftRocketHatch2Position, wristRocketHatch2Position));
		rocketHatch3.whenPressed(new LiftSetPosition(liftRocketHatch3Position, wristRocketHatch3Position));

		// set the buttons at the top of the joystick to intake and home positions
		cargoIntakePosition.whenPressed(new LiftSetPosition(liftCargoIntakePosition, wristCargoIntakePosition));
		homePosition.whenPressed(new LiftSetPosition(liftStartingPosition, wristStartingPosition));

		// control panel
		hatchEject.whenPressed(new HatchEject());
		frontJackRaise.whenPressed(new JackFrontRaise());
		frontJackLower.whenPressed(new JackFrontLower());

	}

	// check if joystick is plugged in
	// if throttle reads a value then we can assume joystick is plugged in
	// if it's plugged in and reads 0 exactly, then we are just unlucky that day
	public boolean joystickIsPluggedIn() {
		return stick.getThrottle() != 0;
	}
}