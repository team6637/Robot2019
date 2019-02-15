/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Jack;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Wrist;

public class Robot extends TimedRobot {

  // Subsystem Instantiations
  public static Drive driveSubsystem = new Drive(false);
  public static Intake intakeSubsystem= new Intake(false);
  public static Lift liftSubsystem = new Lift(true);
  public static Jack jackSubsystem = new Jack();
  public static Limelight limelightSubsystem = new Limelight(true);
  public static Shifter shifterSubsystem = new Shifter();
  public static Wrist wristSubsystem = new Wrist(true);

  public static OI oi;   

  @Override
  public void robotInit() {
    oi = new OI();
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousPeriodic() {
    matchPeriodic();
  }

  @Override
  public void teleopPeriodic() {
    matchPeriodic();
  }

  public void matchPeriodic() {
    Scheduler.getInstance().run();
  }
}
