
package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveManual extends Command {
  public DriveManual() {
    requires(Robot.driveSubsystem);
  }

  @Override
  protected void initialize() {
    Robot.driveSubsystem.leftMaster.setNeutralMode(NeutralMode.Brake);
    Robot.driveSubsystem.rightMaster.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  protected void execute() {
    double move = -Robot.oi.stick.getY();
    double turn = Robot.oi.stick.getTwist();
    Robot.driveSubsystem.manualDrive(move, turn);
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
