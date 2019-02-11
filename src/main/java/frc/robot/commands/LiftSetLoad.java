
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class LiftSetLoad extends Command {
  
  // setup global variable
  String load; 

  // pass in load argument from button
  public LiftSetLoad(String load) {    
    requires(Robot.liftSubsystem);
    this.load = load;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    switch (load) {
      case "Cargo" :
        Robot.liftSubsystem.setCurrentLoadCargo();
        break;
      case "Hatch" : 
        Robot.liftSubsystem.setCurrentLoadHatch();
        break;
    }
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
