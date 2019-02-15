package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class LiftManual extends Command {
  public LiftManual() {
    requires(Robot.liftSubsystem);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {

    SmartDashboard.putNumber("Lift Position", Robot.liftSubsystem.getPosition());

    double liftButton = Robot.oi.stick.getPOV();
    if(liftButton == 0) {
      Robot.liftSubsystem.raise();
    } else if (liftButton == 180) {
      Robot.liftSubsystem.lower();
    } else {
      Robot.liftSubsystem.stop();
    }
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
