package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class JackFront extends Command {

  public JackFront() {
    requires(Robot.jackSubsystem);
  }

  @Override
  protected void initialize() {
    boolean jacked = Robot.jackSubsystem.frontIsJacked;
    if(jacked) {
      Robot.jackSubsystem.frontLower();
      Robot.jackSubsystem.frontIsJacked = false;
    } else {
      Robot.jackSubsystem.frontRaise();
      Robot.jackSubsystem.frontIsJacked = true;
    }
  }

  @Override
  protected void execute() {
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
