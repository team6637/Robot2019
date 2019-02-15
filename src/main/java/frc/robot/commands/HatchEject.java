
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class HatchEject extends Command {

  private int count = 0;
  private boolean retracted = false;

  public HatchEject() {
    requires(Robot.intakeSubsystem);
  }

  @Override
  protected void initialize() {
    Robot.intakeSubsystem.hatchEject();
    this.retracted = false;
    count = 0;
  }

  @Override
  protected void execute() {
          
    if(count > 50) {
      Robot.intakeSubsystem.hatchRetract();
      this.retracted = true;
    }
    count++;
  }

  @Override
  protected boolean isFinished() {
    return this.retracted;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
