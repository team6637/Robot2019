package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.HatchEject;
import frc.robot.commands.IntakeBringIt;
import frc.robot.commands.IntakeSendIt;
import frc.robot.commands.LiftSetLoad;
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

	Button cargoIntakePosition = new JoystickButton(stick, 3);
	Button homePosition = new JoystickButton(stick, 4);

	Button intakeBringIt = new JoystickButton(stick, 2);
	Button intakeSendIt = new JoystickButton(stick, 1);

	Button hatchSendIt = new JoystickButton(stick, 5);


	// setup the control panel
	public Joystick controlPanel = new Joystick(1);

	Button hatchLoadedButton = new JoystickButton(controlPanel, 9);
	Button cargoLoadedButton = new JoystickButton(controlPanel, 10);
	Button frontLifter = new JoystickButton(controlPanel, 11);
	Button rearLifter = new JoystickButton(controlPanel, 12);

	
	public OI() {
		// joystick
		intakeBringIt.whileHeld(new IntakeBringIt());
		intakeSendIt.whileHeld(new IntakeSendIt());

		

		rocketCargo1.whenPressed(new LiftSetPosition(Robot.liftSubsystem.getRocketCargo1Position(), Robot.wristSubsystem.getRocketCargo1Position()));
		rocketCargo2.whenPressed(new LiftSetPosition(Robot.liftSubsystem.getRocketCargo2Position(), Robot.wristSubsystem.getRocketCargo2Position()));	
		rocketCargo3.whenPressed(new LiftSetPosition(Robot.liftSubsystem.getRocketCargo3Position(), Robot.wristSubsystem.getRocketCargo3Position()));
		rocketHatch1.whenPressed(new LiftSetPosition(Robot.liftSubsystem.getRocketHatch1Position(), Robot.wristSubsystem.getRocketHatch1Position()));
		rocketHatch2.whenPressed(new LiftSetPosition(Robot.liftSubsystem.getRocketHatch2Position(), Robot.wristSubsystem.getRocketHatch2Position()));
		rocketHatch3.whenPressed(new LiftSetPosition(Robot.liftSubsystem.getRocketHatch3Position(), Robot.wristSubsystem.getRocketHatch3Position()));

		cargoIntakePosition.whenPressed(new LiftSetPosition(Robot.liftSubsystem.getCargoIntakePosition(), Robot.wristSubsystem.getCargoIntakePosition()));
		homePosition.whenPressed(new LiftSetPosition(Robot.liftSubsystem.getBottomPosition(), Robot.wristSubsystem.getstartingPosition()));
	
		hatchSendIt.whenPressed(new HatchEject());

		// control panel
		cargoLoadedButton.whenPressed(new LiftSetLoad("Cargo"));
		hatchLoadedButton.whenPressed(new LiftSetLoad("Hatch"));
	}

	public boolean joystickOn() {
		return stick.getThrottle() != 0;
	}
}