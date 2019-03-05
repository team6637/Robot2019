package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class JackFrontLower extends Command {
  public JackFrontLower() {
    requires(Robot.jackSubsystem);
  }

  @Override
  protected void initialize() {
    Robot.jackSubsystem.frontRetract();

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