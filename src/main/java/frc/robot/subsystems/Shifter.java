package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.Shift;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Shifter extends Subsystem {

	// setup the states
	enum State {
		HIGH,
		LOW
	}
	
	// initial state
	State state = State.LOW;

	// this is the throttle on the Logitech Joystick
	// the range is -1 to 1
	double shiftSlider;
	
	public DoubleSolenoid solenoid = new DoubleSolenoid(RobotMap.shifterSolenoidLow, RobotMap.shifterSolenoidHigh);
	
	public void shiftLow() {
		solenoid.set(DoubleSolenoid.Value.kReverse);
		state = State.LOW;
	}
	
	public void shiftHigh() {
		solenoid.set(DoubleSolenoid.Value.kForward);
		state = State.HIGH;
	}

    public void initDefaultCommand() {
    	setDefaultCommand(new Shift());
	}
	
	// this is called repeatedly from execute() in a command
	public void periodic() {
		shiftSlider = -Robot.oi.stick.getThrottle();
		if(shiftSlider > 0 && state == State.LOW) {
			shiftHigh();
		} else if (shiftSlider <= 0 && state == State.HIGH) {
			shiftLow();
		}
	}
}