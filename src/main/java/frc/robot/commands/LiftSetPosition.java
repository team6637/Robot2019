/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class LiftSetPosition extends Command {

  int liftTargetPosition = Robot.liftSubsystem.getPosition();
  int wristTargetPosition = Robot.wristSubsystem.getPosition();

  public LiftSetPosition(int liftTargetPosition, int wristTargetPosition) {
    // we're not requiring a subsystem since this command only changes a value in it's target subsytem
    this.liftTargetPosition = liftTargetPosition;
    this.wristTargetPosition = wristTargetPosition;
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.wristSubsystem.setTargetPosition(wristTargetPosition);
    Robot.liftSubsystem.setTargetPosition(liftTargetPosition);
  }

  @Override
  protected boolean isFinished() {
    return true;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {}
}
