package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class LimelightDriverCam extends Command {
  public LimelightDriverCam() {
    requires(Robot.limelightSubsystem);
  }

  @Override
  protected void initialize() {
    Robot.limelightSubsystem.setPipeline(1);
    Robot.limelightSubsystem.setCameraMode(1);
    Robot.limelightSubsystem.setLedMode(1);
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
