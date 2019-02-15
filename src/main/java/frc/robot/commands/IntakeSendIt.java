package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeSendIt extends Command {

  public IntakeSendIt() {
    requires(Robot.intakeSubsystem);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.intakeSubsystem.sendIt();
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    Robot.intakeSubsystem.stop();
  }

  @Override
  protected void interrupted() {
    end();
  }
}
