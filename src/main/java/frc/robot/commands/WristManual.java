package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class WristManual extends Command {
  private boolean tunable;

  public WristManual() {
    requires(Robot.wristSubsystem);

    tunable = Robot.wristSubsystem.tunable;
  }

  @Override
  protected void initialize() {
    Robot.wristSubsystem.resetPosition();
  }

  @Override
  protected void execute() {
    double currentPosition = Robot.wristSubsystem.getPosition();

    if(tunable) {
      SmartDashboard.putNumber("Wrist Position", currentPosition);
      SmartDashboard.putNumber("Wrist Velocity", Robot.wristSubsystem.getCurrentVelocity());
    }
    
     //double move = -Robot.oi.stick.getY();
     //Robot.wristSubsystem.move(move);

    // SmartDashboard.putNumber("Wrist Move", move);

    double joystickX = Robot.oi.controlPanel.getX();

    if(joystickX == -1) {
      Robot.wristSubsystem.raise();
    } else if(joystickX == 1) {
      Robot.wristSubsystem.lower();
    } else {
      Robot.wristSubsystem.stop();
    }
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    Robot.wristSubsystem.stop();
  }

  @Override
  protected void interrupted() {
  }
}