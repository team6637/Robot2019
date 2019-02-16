package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.RobotState.State;

public class LiftSetPosition extends Command {

  int liftTargetPosition, wristTargetPosition;

  State currentRobotState;
  State targetRobotState;

  int liftStartingPosition, liftCargoIntakePosition, liftRocketCargo1Position, liftRocketCargo2Position, 
    liftRocketCargo3Position, liftRocketHatch1Position, liftRocketHatch2Position, liftRocketHatch3Position;

  int wristStartingPosition, wristCargoIntakePosition, wristRocketCargo1Position, wristRocketCargo2Position,
    wristRocketCargo3Position, wristRocketHatch1Position, wristRocketHatch2Position, wristRocketHatch3Position;

  boolean isComplete, isFirstMover;

  public LiftSetPosition(State targetRobotState) {
    // we're not requiring a subsystem since this command only changes a value in it's target subsytem
    
    this.targetRobotState = targetRobotState;
    this.currentRobotState = RobotState.currentState;

    // setup variables to hold the preset lift positions
		liftStartingPosition = Robot.liftSubsystem.getStartingPosition();
		liftCargoIntakePosition = Robot.liftSubsystem.getCargoIntakePosition();
		liftRocketCargo1Position = Robot.liftSubsystem.getRocketCargo1Position();
		liftRocketCargo2Position = Robot.liftSubsystem.getRocketCargo2Position();
		liftRocketCargo3Position = Robot.liftSubsystem.getRocketCargo3Position();
		liftRocketHatch1Position = Robot.liftSubsystem.getRocketHatch1Position();
		liftRocketHatch2Position = Robot.liftSubsystem.getRocketHatch2Position();
		liftRocketHatch3Position = Robot.liftSubsystem.getRocketHatch3Position();

		// setup variables to hold the preset wrist positions
		wristStartingPosition = Robot.wristSubsystem.getStartingPosition();
		wristCargoIntakePosition = Robot.wristSubsystem.getCargoIntakePosition();
		wristRocketCargo1Position = Robot.wristSubsystem.getRocketCargo1Position();
		wristRocketCargo2Position = Robot.wristSubsystem.getRocketCargo2Position();
		wristRocketCargo3Position = Robot.wristSubsystem.getRocketCargo3Position();
		wristRocketHatch1Position = Robot.wristSubsystem.getRocketHatch1Position();
		wristRocketHatch2Position = Robot.wristSubsystem.getRocketHatch2Position();
    wristRocketHatch3Position = Robot.wristSubsystem.getRocketHatch3Position();
    
    this.isComplete = false;
    this.isFirstMover = true;
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {

    switch(this.targetRobotState) {


      // to HOME
      case HOME:

        // moving to HOME from CARGO_INTAKE
        if(currentRobotState == State.CARGO_INTAKE) {
          if(this.isFirstMover) {
            Robot.wristSubsystem.setTargetPosition(this.wristStartingPosition);
            this.isFirstMover = false;
          } else {
            if(Robot.wristSubsystem.getPosition() > 700) {
              Robot.liftSubsystem.setTargetPosition(this.liftStartingPosition);
              this.isComplete = true;
            }
          }

        // moving to HOME from CARGO_3 or HATCH_3
        } else if (this.currentRobotState == State.CARGO_3 || this.currentRobotState == State.HATCH_3){
          if(this.isFirstMover) {
            Robot.liftSubsystem.setTargetPosition(this.liftStartingPosition);
            this.isFirstMover = false;
          } else {
            if(Robot.liftSubsystem.getPosition() < 2500) {
              Robot.wristSubsystem.setTargetPosition(this.wristStartingPosition);
              this.isComplete = true;
            }
          }

        // moving to HOME from all other positions
        } else {
          Robot.wristSubsystem.setTargetPosition(this.wristStartingPosition);
          Robot.liftSubsystem.setTargetPosition(this.liftStartingPosition);
          this.isComplete = true;
        }
        break;

      
      // to CARGO INTAKE
      case CARGO_INTAKE:

        // moving to CARGO_INTAKE
        Robot.wristSubsystem.setTargetPosition(this.wristCargoIntakePosition);
        Robot.liftSubsystem.setTargetPosition(this.liftCargoIntakePosition);
        this.isComplete = true;
        break;


      // to CARGO 1
      case CARGO_1:

        // moving to CARGO_1 from CARGO_INTAKE
        if(currentRobotState == State.CARGO_INTAKE) {
          if(this.isFirstMover) {
            Robot.wristSubsystem.setTargetPosition(this.wristRocketCargo1Position);
            this.isFirstMover = false;
          } else {
            if(Robot.wristSubsystem.getPosition() > 700) {
              Robot.liftSubsystem.setTargetPosition(this.liftRocketCargo1Position);
              this.isComplete = true;
            }
          }

        // moving to CARGO_1 from all other positions
        } else {
          Robot.wristSubsystem.setTargetPosition(this.wristRocketCargo1Position);
          Robot.liftSubsystem.setTargetPosition(this.liftRocketCargo1Position);
          this.isComplete = true;
        }
        break;


      // to CARGO 2
      case CARGO_2:

        // moving to CARGO_2 from all positions
        Robot.wristSubsystem.setTargetPosition(this.wristRocketCargo2Position);
        Robot.liftSubsystem.setTargetPosition(this.liftRocketCargo2Position);
        this.isComplete = true;
        break;
      

      // to CARGO 3
      case CARGO_3:

        // moving to CARGO_3 from all positions
        Robot.wristSubsystem.setTargetPosition(this.wristRocketCargo3Position);
        Robot.liftSubsystem.setTargetPosition(this.liftRocketCargo3Position);
        this.isComplete = true;
        break;


      // to HATCH 1
      case HATCH_1:

        // moving to HATCH_1 from CARGO_INTAKE
        if(currentRobotState == State.CARGO_INTAKE) {
          if(this.isFirstMover) {
            Robot.wristSubsystem.setTargetPosition(this.wristRocketHatch1Position);
            this.isFirstMover = false;
          } else {
            if(Robot.wristSubsystem.getPosition() > 700) {
              Robot.liftSubsystem.setTargetPosition(this.liftRocketHatch1Position);
              this.isComplete = true;
            }
          }

        // moving to HATCH_1 from all other positions
        } else {
          Robot.wristSubsystem.setTargetPosition(this.wristRocketHatch1Position);
          Robot.liftSubsystem.setTargetPosition(this.liftRocketHatch1Position);
          this.isComplete = true;
        }
        break;


      // to HATCH 2
      case HATCH_2:

        // moving to HATCH_2 from all positions
        Robot.wristSubsystem.setTargetPosition(this.wristRocketHatch2Position);
        Robot.liftSubsystem.setTargetPosition(this.liftRocketHatch2Position);
        this.isComplete = true;
        break;


      // to HATCH 3
      case HATCH_3:

        // moving to HATCH_3 from all positions
        Robot.wristSubsystem.setTargetPosition(this.wristRocketHatch2Position);
        Robot.liftSubsystem.setTargetPosition(this.liftRocketHatch2Position);
        this.isComplete = true;
        break;
    }
  }

  @Override
  protected boolean isFinished() {
    return isComplete;
  }

  @Override
  protected void end() {
    this.currentRobotState = this.targetRobotState;
    RobotState.currentState = this.targetRobotState;
  }

  @Override
  protected void interrupted() {}
}
