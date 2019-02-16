package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class Jack extends Subsystem {

  public DoubleSolenoid jackFront = new DoubleSolenoid(RobotMap.jackFrontLow, RobotMap.jackFrontHigh);
  public DoubleSolenoid jackRear = new DoubleSolenoid(RobotMap.jackRearLow, RobotMap.jackRearHigh);

  public Jack() {
    frontLower();
  }

  // raise the robot
  public void frontRaise() {
    jackFront.set(DoubleSolenoid.Value.kReverse);
  }

  // lower the robot back down
  public void frontLower() {
    jackFront.set(DoubleSolenoid.Value.kForward);
  }

  public void rearRaise() {
    jackRear.set(DoubleSolenoid.Value.kForward);
  }

  public void rearLower() {
    jackRear.set(DoubleSolenoid.Value.kReverse);
  }
  
  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new JackFront());
  }
}
