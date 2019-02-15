package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class WristMM extends Command {
  public WristMM() {
    requires(Robot.wristSubsystem);
  }

  @Override
  protected void initialize() {
    Robot.wristSubsystem.initializer();
  }

  @Override
  protected void execute() {
    Robot.wristSubsystem.periodicLoop();
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