package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotState;

public class LimelightAutoAlign extends Command {

  public double kPMove_hatch = .26;
  public double kPTurn_hatch = .06;
  public double targetArea_hatch = 2.659;
  public double maxSpeed = .6;

  public double kPTurn_cargo = .07;
  public double targetArea_cargo = 3;

  public LimelightAutoAlign() {
    requires(Robot.limelightSubsystem);
    requires(Robot.driveSubsystem);

    SmartDashboard.putNumber("Camera Move kP", kPMove_hatch);
    SmartDashboard.putNumber("Camera Turn kP", kPTurn_hatch);
    SmartDashboard.putNumber("Camera maxSpeed", maxSpeed);
  
    SmartDashboard.putNumber("Camera Turn kP for Cargo", kPTurn_cargo);
  }

  @Override
  protected void initialize() {

    if(RobotState.currentState.name() == "CARGO_INTAKE") {
      Robot.limelightSubsystem.setPipeline(2);
      Robot.limelightSubsystem.setCameraMode(0);
    } else {
      Robot.limelightSubsystem.setPipeline(0);
      Robot.limelightSubsystem.setCameraMode(0);
    }

    Robot.driveSubsystem.leftMaster.setNeutralMode(NeutralMode.Coast);
    Robot.driveSubsystem.rightMaster.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  protected void execute() {
    
    boolean isTarget = Robot.limelightSubsystem.isTarget();
    SmartDashboard.putBoolean("Camera has target", isTarget);

    if(isTarget) {

      double tx = Robot.limelightSubsystem.getTx();
      double ty = Robot.limelightSubsystem.getTy();
      double ta = Robot.limelightSubsystem.getTa();

      SmartDashboard.putNumber("Camera tx", tx);
      SmartDashboard.putNumber("Camera ty", ty);
      SmartDashboard.putNumber("Camera ta", ta);

      if(RobotState.currentState.name() == "CARGO_INTAKE") {

        kPTurn_cargo = SmartDashboard.getNumber("Camera Cargo Turn kP", kPTurn_cargo);

        double turnAlign = tx * kPTurn_hatch;
        double move = -Robot.oi.stick.getY();
        Robot.driveSubsystem.manualDrive(move, turnAlign);

        Robot.intakeSubsystem.bringIt();

      } else {

        maxSpeed = SmartDashboard.getNumber("Camera maxSpeed", maxSpeed);

        kPMove_hatch = SmartDashboard.getNumber("Camera Move kP", kPMove_hatch);
        kPTurn_hatch = SmartDashboard.getNumber("Camera Turn kP", kPTurn_hatch);

        double turnAlign = tx * kPTurn_hatch;

        double moveAlign = (targetArea_hatch - ta) * kPMove_hatch;
        if(moveAlign > maxSpeed) moveAlign = maxSpeed;

        // too close
        if(ta > targetArea_hatch) {
          Robot.driveSubsystem.stop();

        // go forward 
        } else {
          Robot.driveSubsystem.manualDrive(moveAlign, turnAlign);
        }
      }

    } else {
      Robot.driveSubsystem.stop();
    }
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