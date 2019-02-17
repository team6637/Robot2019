package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class LimelightAlignToHatch extends Command {

  public double kPMove = .11;
  public double kPTurn = .06;
  public double targetArea = 6.4;
  public double maxSpeed = .6;

  public LimelightAlignToHatch() {
    requires(Robot.limelightSubsystem);
    requires(Robot.driveSubsystem);

    SmartDashboard.putNumber("Camera Move kP", kPMove);
    SmartDashboard.putNumber("Camera Turn kP", kPTurn);
    SmartDashboard.putNumber("Camera maxSpeed", maxSpeed);
  }

  @Override
  protected void initialize() {
    Robot.limelightSubsystem.setPipeline(0);
    Robot.limelightSubsystem.setCameraMode(0);
    Robot.limelightSubsystem.setLedMode(0);

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

      maxSpeed = SmartDashboard.getNumber("Camera maxSpeed", maxSpeed);


      kPMove = SmartDashboard.getNumber("Camera Move kP", kPMove);
      kPTurn = SmartDashboard.getNumber("Camera Turn kP", kPTurn);

      double turnAlign = tx * kPTurn;

      double moveAlign = (targetArea - ta) * kPMove;
      if(moveAlign > maxSpeed) moveAlign = maxSpeed;

      // too close
      if(ta > targetArea) {
        Robot.driveSubsystem.stop();

      // go forward 
      } else {
        Robot.driveSubsystem.manualDrive(moveAlign, turnAlign);
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
  }

  @Override
  protected void interrupted() {
  }
}
