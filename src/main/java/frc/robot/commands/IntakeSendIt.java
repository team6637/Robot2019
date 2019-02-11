/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeSendIt extends Command {

  String liftLoad;

  public IntakeSendIt() {
    requires(Robot.intakeSubsystem);
  }

  @Override
  protected void initialize() {

    // get current load state from lift
    liftLoad = Robot.liftSubsystem.getLoad();
    
    // if the lift has a hatch, eject it
    if(liftLoad == "Hatch") {
      Robot.intakeSubsystem.hatchEject();      
      Timer.delay(1);
      Robot.intakeSubsystem.hatchRetract();
    }

  }

  @Override
  protected void execute() {

    // if the lift didn't have the hatch at start, spin the ball collector out
    if(liftLoad != "Hatch")
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
