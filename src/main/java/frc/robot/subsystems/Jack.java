package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class Jack extends Subsystem {

  public DoubleSolenoid jackFront = new DoubleSolenoid(RobotMap.jackFrontLow, RobotMap.jackFrontHigh);
  public DoubleSolenoid jackRear = new DoubleSolenoid(RobotMap.jackRearLow, RobotMap.jackRearHigh);

  public Jack() {
    frontRetract();
    rearRetract();
  }

  // raise the robot
  public void frontRaise() {
    jackFront.set(DoubleSolenoid.Value.kForward);
  }

  // lower the robot back down
  public void frontRetract() {
    jackFront.set(DoubleSolenoid.Value.kReverse);
  }

  public void rearRaise() {
    jackRear.set(DoubleSolenoid.Value.kForward);
  }

  public void rearRetract() {
    jackRear.set(DoubleSolenoid.Value.kReverse);
  }
  
  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new JackFront());
  }
}
