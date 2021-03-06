package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class Shift extends Command {
  public Shift() {
    requires(Robot.shifterSubsystem);
  }

  @Override
  protected void initialize() {
    Robot.shifterSubsystem.shiftLow();
  }

  @Override
  protected void execute() {
    Robot.shifterSubsystem.periodicLoop();
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