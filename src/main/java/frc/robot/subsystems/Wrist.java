
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.WristMM;
import frc.robot.util.Gains;


public class Wrist extends Subsystem {
  
  public boolean tunable;

  // how far does the encoder have to travel to get from starting position to horizontal
  private static int horizontalOffset = 700;

  // how many degrees has the arm traveled to get to the horizontal position from start position
  private double horizontalAngleDisplacement = 68;

  // what is the least output required to hold the arm up in the horizontal position
  private double  horizontalHoldOutput = 0.03;

  // setup predefined setpoints
  private int startingPosition = 2000;
  private int cargoIntakePosition = 55;
  private int cargoBayPosition = 1195;
  private int rocketCargo1Position = 1318;
  private int rocketCargo2Position = 550;
  private int rocketCargo3Position = 262;
  private int rocketHatch1Position = 1152;
  private int rocketHatch2Position = 663;
  private int rocketHatch3Position = 350;

  private int targetPosition = startingPosition;
  private int lastExecutedPosition;
  private final static int onTargetThreshold = 100;

  private int manualIncrement = 8;

  // define soft limits
  private int upPositionLimit = 2000;
  private int downPositionLimit = 0;

  // PID slots
  public final static int slot_up = 0;
  public final static int slot_down = 1;

  // Gains
  Gains upGains, downGains;

  // Motion Parameters 
  public int maxVelocityUp = 300;
  public int maxAccelerationUp = 300;
  public int maxVelocityDown = 300;
  public int maxAccelerationDown = 300;

  public WPI_TalonSRX motor;

  public Wrist(boolean tunable) {
    this.tunable = tunable;

    motor = new WPI_TalonSRX(RobotMap.wristMotorPort);

    upGains = new Gains(3, 0, 0, 0, 150, 1);
    downGains = new Gains(3, 0, 0, 0, 150, 1);

    // reset talons
    motor.configFactoryDefault();

    motor.setInverted(false);
    motor.setNeutralMode(NeutralMode.Coast);

    //Set up encoder
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    motor.setSensorPhase(false);
    
    motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, RobotMap.timeoutMs);
    motor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, RobotMap.timeoutMs);
    motor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, RobotMap.timeoutMs);
    motor.configPeakOutputForward(+1.0, RobotMap.timeoutMs);
    motor.configPeakOutputReverse(-1.0, RobotMap.timeoutMs);
    
    // set speeds
    motor.configMotionAcceleration(maxAccelerationDown, RobotMap.timeoutMs);
    motor.configMotionCruiseVelocity(maxVelocityDown, RobotMap.timeoutMs);
    
    // setup a PIDF slot for going up
    motor.config_kP(slot_up, upGains.kP, RobotMap.timeoutMs);
    motor.config_kI(slot_up, upGains.kI, RobotMap.timeoutMs);
    motor.config_kD(slot_up, upGains.kD, RobotMap.timeoutMs);
    motor.config_kF(slot_up, upGains.kF, RobotMap.timeoutMs);
    motor.config_IntegralZone(slot_up, upGains.kIZone, RobotMap.timeoutMs);
    motor.configClosedLoopPeakOutput(slot_up, upGains.kPeakOutput, RobotMap.timeoutMs);
    
    // setup a PIDF slot for going down
    motor.config_kP(slot_down, downGains.kP, RobotMap.timeoutMs);
    motor.config_kI(slot_down, downGains.kI, RobotMap.timeoutMs);
    motor.config_kD(slot_down, downGains.kD, RobotMap.timeoutMs);
    motor.config_kF(slot_down, downGains.kF, RobotMap.timeoutMs);
    motor.config_IntegralZone(slot_down, downGains.kIZone, RobotMap.timeoutMs);
    motor.configClosedLoopPeakOutput(slot_down, downGains.kPeakOutput, RobotMap.timeoutMs);
    
    int closedLoopTimeMs = 1;
    motor.configClosedLoopPeriod(slot_up, closedLoopTimeMs, RobotMap.timeoutMs);
    motor.configClosedLoopPeriod(slot_down, closedLoopTimeMs, RobotMap.timeoutMs);
    
    // set profile slot, this will be updated when gains need to change
    motor.selectProfileSlot(slot_down, RobotMap.pidPrimary);

    // Current Limiting
    motor.configPeakCurrentLimit(RobotMap.current30AmpPeakCurrentLimit, RobotMap.timeoutMs);
    motor.configPeakCurrentDuration(RobotMap.current30AmpPeakCurrentDuration, RobotMap.timeoutMs);
    motor.configContinuousCurrentLimit(RobotMap.current30AmpContinuousCurrentLimit, RobotMap.timeoutMs);
    motor.enableCurrentLimit(true); 

    // set encoder and set point to starting position
    resetPosition();
    setTargetPosition(getStartingPosition());
  }

  // ENCODER
  public int getPosition() {
    return motor.getSelectedSensorPosition();
  }

  public void resetPosition() {
    motor.setSelectedSensorPosition(2000, 0, 10);
  }
  
  // get velocity
  public double getCurrentVelocity() {
    return motor.getSelectedSensorVelocity();
  }

  @Override
	public void initDefaultCommand() {
    setDefaultCommand(new WristMM());
  }


  // MANUAL CONTROL
  public void raise() {
    motor.set(ControlMode.PercentOutput, 1);
  }
  public void lower() {
    motor.set(ControlMode.PercentOutput, -1);
  }
  public void move(double move) {
    motor.set(ControlMode.PercentOutput, move);
  }
  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
  }


  

  // MOTION MAGIC CONTROL
  // set MM as default command to manualControlStick with Motion Magic
  // uses targetPostion to move to a setpoint
  public void motionMagicControl() {
    
    // print a random number to visually show when MM restarts
    if(tunable)
      SmartDashboard.putNumber("Wrist Motion Control Starting", Math.random());

    // setup gain slot, velocity and acceleration
    // since kP and accel values are the same, we don't need this
    //manageMotion(targetPosition);

    // Percent to add to Closed Loop Output
    //double feedForward = getFeedForward();
    // temporarily disable feed forward until PID is tuned
    //double feedForward = 0;

    // Do It!!!
		motor.set(ControlMode.MotionMagic, targetPosition);

    // keep track so we know when targetPosition has changed (in periodic method)
    lastExecutedPosition = targetPosition;
  }

  // setup gain slot, velocity and acceleration
  public void manageMotion(int targetPosition) {
    int currentPosition = getPosition();

    // going up
    if(currentPosition < targetPosition) {

      // set accel and velocity for going up
      motor.configMotionAcceleration(maxAccelerationUp, RobotMap.timeoutMs);
      motor.configMotionCruiseVelocity(maxVelocityUp, RobotMap.timeoutMs);

      // select the up gains
      motor.selectProfileSlot(slot_up, RobotMap.pidPrimary);

    } else {
      
      // set accel and velocity for going down
      motor.configMotionAcceleration(maxAccelerationDown, RobotMap.timeoutMs);
      motor.configMotionCruiseVelocity(maxVelocityDown, RobotMap.timeoutMs);

      // select the down gains
      motor.selectProfileSlot(slot_down, RobotMap.pidPrimary);
    }

  }

  public int getStartingPosition() {
    return this.startingPosition;
  }
  public int getCargoIntakePosition() {
    return this.cargoIntakePosition;
  }
  public int getCargoBayPosition() {
    return this.cargoBayPosition;
  }
  public int getRocketCargo1Position() {
    return this.rocketCargo1Position;
  }
  public int getRocketCargo2Position() {
    return this.rocketCargo2Position;
  }
  public int getRocketCargo3Position() {
    return this.rocketCargo3Position;
  }
  public int getRocketHatch1Position() {
    return this.rocketHatch1Position;
  }
  public int getRocketHatch2Position() {
    return this.rocketHatch2Position;
  }
  public int getRocketHatch3Position() {
    return this.rocketHatch3Position;
  }

  public int getTargetPosition() {
    return this.targetPosition;
  }

  public boolean setTargetPosition(int targetPosition) {
    if(isPositionValid(targetPosition)) {
      this.targetPosition = targetPosition;

      SmartDashboard.putNumber("Wrist Target", targetPosition);

      return true;
    } 
    return false;
  }

  public void incrementTargetPosition(int increment) {
    setTargetPosition(this.targetPosition + increment);
  }

  // check if requested target position is within valid limits
  public boolean isPositionValid(int targetPosition) {
    return(targetPosition >= downPositionLimit && targetPosition <= upPositionLimit);
  }

  // check whether the manualControlStick is in position
  public boolean isInPosition(int targetPosition) {
    int currentPosition = getPosition();
    int positionError = Math.abs(targetPosition - currentPosition);
    return positionError < onTargetThreshold;
  }

  // return the angle of the wrist relative to the lift based on the current encoder value
  public double getRelativeAngle () {
    int currentPosition = getPosition();

    // divide the encoder position when arm is horizontal by the angle displacement
    // so if you moved the arm 30 degrees and read 1000 ticks, it would be 1000/30 ticks per degree
    double ticksPerDegree = horizontalOffset/horizontalAngleDisplacement;
    double relativeAngle = (currentPosition - horizontalOffset) / ticksPerDegree;
    return relativeAngle;
  }

  // absolute angle (angle in relation to the floor)
  public double getAngle() {
    double relativeAngle = getRelativeAngle();
    double absoluteAngle = relativeAngle + Robot.liftSubsystem.getAngle();
    return absoluteAngle;
  }
  
  // take the horizontalHoldOutput and linearize it by multiplying it by the cos of the angle of the arm
  public double getFeedForward() {
    double radians = Math.toRadians(getAngle());
    double feedForward = horizontalHoldOutput * Math.cos(radians);

    // set feedforward to 0 in home position so we aren't giving the motor voltage when it's just sitting there
    if(getRelativeAngle() > 115)
      feedForward = 0;
    
    return feedForward;
  }

  public void initializer() {

    //resetPosition();
    //setTargetPosition(getStartingPosition());

    if(tunable) {
      SmartDashboard.putNumber("Wrist Target", getStartingPosition());
      SmartDashboard.putNumber("Wrist Position", this.getPosition());
      SmartDashboard.putNumber("Wrist Velocity", this.getCurrentVelocity());
      SmartDashboard.putNumber("Wrist Closed Loop Error", this.motor.getClosedLoopError());

      SmartDashboard.putNumber("Wrist Velocity Up", maxVelocityUp);
      SmartDashboard.putNumber("Wrist Velocity Down", maxVelocityDown);
      SmartDashboard.putNumber("Wrist Acceleration Up", maxAccelerationUp);
      SmartDashboard.putNumber("Wrist Acceleration Down", maxAccelerationDown);

      SmartDashboard.putNumber("Wrist Up kP", upGains.kP);
      SmartDashboard.putNumber("Wrist Down kP", downGains.kP);

      SmartDashboard.putNumber("Wrist Manual Increment", manualIncrement);
    }
  }

  // to tune, call this in the execute method of a command
  public void periodicLoop() {
    boolean changed = false;

    SmartDashboard.putNumber("Wrist Position", this.getPosition());

    // if targetPosition has changed since MM was last called, call MM again
    if(lastExecutedPosition != targetPosition)
      changed = true;

    // check for manual control
    double manualControlStick = Robot.oi.controlPanel.getX();
    if(manualControlStick == 1) {
      incrementTargetPosition(manualIncrement);
      changed = true;
    } else if (manualControlStick == -1) {
      incrementTargetPosition(-manualIncrement);
      changed = true;
    }

    // tune from Smart Dashboard
    if(tunable) {
      SmartDashboard.putNumber("Wrist Velocity", this.getCurrentVelocity());
      SmartDashboard.putNumber("Wrist Relative Angle", getRelativeAngle());
      SmartDashboard.putNumber("Wrist Absolute Angle", getAngle());
      SmartDashboard.putNumber("Wrist FeedForward", getFeedForward()); 
  
      int sdManualIncrement = (int) SmartDashboard.getNumber("Wrist Manual Increment", manualIncrement);
      if(sdManualIncrement != manualIncrement) 
        manualIncrement = sdManualIncrement;

      int sdVelUp = (int) SmartDashboard.getNumber("Wrist Velocity Up", maxVelocityUp);
      if(sdVelUp != maxVelocityUp) 
        this.maxVelocityUp = sdVelUp;

      int sdVelDown = (int) SmartDashboard.getNumber("Wrist Velocity Down", maxVelocityDown);
      if(sdVelDown != maxVelocityDown) 
        this.maxVelocityDown = sdVelDown;

      int sdAccelUp = (int) SmartDashboard.getNumber("Wrist Acceleration Up", maxAccelerationUp);
      if(sdAccelUp != maxAccelerationUp)
        this.maxAccelerationUp = sdAccelUp;

      int sdAccelDown = (int) SmartDashboard.getNumber("Wrist Accerleration Down", maxAccelerationDown);
      if(sdAccelDown != maxAccelerationDown)
        this.maxAccelerationDown = sdAccelDown;

      double sdUpP = SmartDashboard.getNumber("Wrist Up kP", upGains.kP);
      if(sdUpP != upGains.kP) {
        upGains.kP = sdUpP;
        motor.config_kP(slot_up, upGains.kP, RobotMap.timeoutMs);
        changed = true;
      }

      double sdDownP = SmartDashboard.getNumber("Wrist Down kP", downGains.kP);
      if(sdDownP != downGains.kP) {
        downGains.kP = sdDownP;
        motor.config_kP(slot_down, downGains.kP, RobotMap.timeoutMs);
      }
      
      int sdTargetPosition = (int) SmartDashboard.getNumber("Wrist Target", targetPosition);
      if(sdTargetPosition != this.targetPosition) {
        this.setTargetPosition(sdTargetPosition);
        changed = true;
      }
    }

    // run motion magic
    if(changed) {
      this.motionMagicControl();
    }
  }  
}
