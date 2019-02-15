package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class LiftMM extends Command {

  public LiftMM() {
    requires(Robot.liftSubsystem);
  }

  @Override
  protected void initialize() {
    Robot.liftSubsystem.initializer();
  }

  @Override
  protected void execute() {
    Robot.liftSubsystem.periodicLoop();
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