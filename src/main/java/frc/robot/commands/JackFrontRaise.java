package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class JackFrontRaise extends Command {
  public JackFrontRaise() {
    requires(Robot.jackSubsystem);
  }

  @Override
  protected void initialize() {
    Robot.jackSubsystem.frontRaise();
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