package frc.robot;

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
  public static int jackFrontLow = 4;
  public static int jackFrontHigh = 5;
  public static int jackRearLow = 6;
  public static int jackRearHigh = 7;
  
  // USB
  public static int joystickPort = 0;
  public static int controlPort = 1;

  // DIO
  public static final int liftLimitSwitchPort = 1;
  public static final int wristLimitSwitchPort = 2;

  // TALON SETTINGS
  public static final int slot0 = 0;
  public static final int slot1 = 1;
  public static final int slot2 = 2;
  public static final int slot3 = 3;

  public static final int pidPrimary = 0;
  public static final int pidTurn = 1;

  public static final int timeoutMs = 30;
  public static final int baseTrajPeriodMs = 0;
  
  public static final double driveNeutralDeadband = 0.1;

  // LIFT SETTINGS
  public static final int liftSensorUnitsPerRotation = 1440;

  // 30AMP CURRENT LIMITS
  public static final int current30AmpPeakCurrentLimit = 25;
  public static final int current30AmpPeakCurrentDuration = 200;
  public static final int current30AmpContinuousCurrentLimit = 25;

  // 40AMP CURRENT LIMITS
  public static final int current40AmpPeakCurrentLimit = 35;
  public static final int current40AmpPeakCurrentDuration = 200;
  public static final int current40AmpContinuousCurrentLimit = 35;
}