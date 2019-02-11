/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  // CAN
  public static int leftMasterPort = 1;
  public static int leftSlave1Port = 2;
  public static int leftSlave2Port = 3;
  public static int rightMasterPort = 4;
  public static int rightSlave1Port = 5;
  public static int rightSlave2Port = 6;

  public static int liftMasterPort = 7; 
  public static int liftSlavePort = 8;

  public static int wristMotorPort = 9;
  public static int intakeMotorPort = 10;
  
  // PCM
  public static int shifterSolenoidLow = 0;
  public static int shifterSolenoidHigh = 1;
  public static int intakeSolenoidLow = 2;
  public static int intakeSolenoidHigh = 3;
  
  // USB
  public static int joystickPort = 0;
  public static int controlPort = 1;

  // PWM

  // ANALOG
  public static int wristPot = 3;

  // DIO
  public static final int liftLimitSwitchPort = 1;
  public static final int wristLimitSwitchPort = 2;
  
  // DRIVETRAIN
  public static final double wheelDiameter = 6.048;
  public static final double actualWheelCircumference = 19.25; // inches
  public static final double driveGearRatio = 3;
  public static final double mysteryScalingFactor = 1 - 3.5 / 60;

  //public static final int sensorUnitsPerRotation = 1440;
  public static final int sensorUnitsPerRotation = 480; // 1440 / 3, 480 ticks of the sensor equal 1 rev of the wheels
  public static final int driveCruiseVelocity = 500;
  public static final int driveAcceleration = 800;


  // LIFT VALUES
  public static final int liftSensorUnitsPerRotation = 1440;

  // GENERAL TALON CONSTANTS
  /**
   * Using the configSelectedFeedbackCoefficient() function, scale units to 3600 per rotation.
   * This is nice as it keeps 0.1 degrees of resolution, and is fairly intuitive.
   */
  public final static double turnTravelUnitsPerRotation = 3600;

  /**
   * This is a property of the Pigeon IMU, and should not be changed.
   */
  public final static int pigeonUnitsPerRotation = 8192;

  /**
   * Which PID slot to pull gains from. Starting 2018, you can choose from
   * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
   * configuration.
   */
  public static final int slot0 = 0;
  public static final int slot1 = 1;
  public static final int slot2 = 2;
  public static final int slot3 = 3;

  public static final int pidPrimary = 0;
  public static final int pidTurn = 1;

  public static final int remote_0 = 0;
  public static final int remote_1 = 1;

  /**
   * set to zero to skip waiting for confirmation, set to nonzero to wait and
   * report to DS if action fails.
   */
  public static final int timeoutMs = 30;

  /**
   * Base trajectory period to add to each individual trajectory point's
   * unique duration. This can be set to any value within [0,255]ms.
   */
  public static final int baseTrajPeriodMs = 0;

  /**
   * Motor deadband, set to 1%.
   */
  public static final double neutralDeadband = 0.01;

}