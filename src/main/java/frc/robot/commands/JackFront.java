package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class JackFront extends Command {

  public static boolean jacked = false;
  
  public JackFront() {
    requires(Robot.jackSubsystem);
  }

  @Override
  protected void initialize() {    
    Robot.jackSubsystem.frontRaise();
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
    Robot.jackSubsystem.frontRetract();
  }

  @Override
  protected void interrupted() {
    end();
  }
}
