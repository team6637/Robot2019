package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class LimelightListen extends Command {
  public LimelightListen() {
    requires(Robot.limelightSubsystem);
  }

  @Override
  protected void initialize() {
    Robot.limelightSubsystem.setLedMode(1);
    Robot.limelightSubsystem.setCameraMode(1);
  }

  @Override
  protected void execute() {
    boolean isTarget = Robot.limelightSubsystem.isTarget();
    SmartDashboard.putBoolean("Camera has target", isTarget);
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
