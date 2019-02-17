package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.HatchEject;
import frc.robot.commands.IntakeBringIt;
import frc.robot.commands.IntakeSendIt;
import frc.robot.commands.JackFront;
import frc.robot.commands.JackRear;
import frc.robot.commands.LiftSetPosition;
import frc.robot.commands.LimelightAlignToHatch;

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
	Button alignWithCamera = new JoystickButton(controlPanel, 10);
	Button frontJack = new JoystickButton(controlPanel, 11);
	Button rearJack = new JoystickButton(controlPanel, 12);

	public OI() {
		// joystick
		intakeBringIt.whileHeld(new IntakeBringIt());
		intakeSendIt.whileHeld(new IntakeSendIt());

		// pass the position variables on the 6 side buttons to the LiftSetPosition Command
		rocketCargo1.whenPressed(new LiftSetPosition(RobotState.State.CARGO_1));
		rocketCargo2.whenPressed(new LiftSetPosition(RobotState.State.CARGO_2));	
		rocketCargo3.whenPressed(new LiftSetPosition(RobotState.State.CARGO_3));
		rocketHatch1.whenPressed(new LiftSetPosition(RobotState.State.HATCH_1));
		rocketHatch2.whenPressed(new LiftSetPosition(RobotState.State.HATCH_2));
		rocketHatch3.whenPressed(new LiftSetPosition(RobotState.State.HATCH_3));

		// set the buttons at the top of the joystick to intake and home positions
		cargoIntakePosition.whenPressed(new LiftSetPosition(RobotState.State.CARGO_INTAKE));
		homePosition.whenPressed(new LiftSetPosition(RobotState.State.HOME));

		// control panel
		hatchEject.whenPressed(new HatchEject());
		alignWithCamera.whileHeld(new LimelightAlignToHatch());
		frontJack.whenPressed(new JackFront());
		rearJack.whenPressed(new JackRear());

	}

	// check if joystick is plugged in
	// if throttle reads a value then we can assume joystick is plugged in
	// if it's plugged in and reads 0 exactly, then we are just unlucky that day
	public boolean joystickIsPluggedIn() {
		return stick.getThrottle() != 0;
	}
}