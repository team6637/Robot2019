package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.LiftMM;
import frc.robot.util.Gains;

public class Lift extends Subsystem {

  private boolean tunable;

  // what does the encoder read when the arm is horizontal
  private int horizontalPosition = 2500;

  // how many degrees has the arm traveled to get to the horizontal position from start position
  private double horizontalAngleDisplacement = 61;

  // what is the least output required to hold the arm up in the horizontal position
  private double horizontalHoldOutput = 0.125;

  // setup predefined setpoints
  private int startingPosition = 0;
  private int cargoIntakePosition = 1680;
  private int cargoBayPosition = 1969;
  private int rocketCargo1Position = 0;
  private int rocketCargo2Position = 2760;
  private int rocketCargo3Position = 4500;
  private int rocketHatch1Position = 0;
  private int rocketHatch2Position = 2638;
  private int rocketHatch3Position = 5167;
  

  // starting position
  // lastExecutedPosition from MM, compared to targetPosition in periodicLoop() method
  // threshold: defines if arm is close enough to setpoint to be considered in proper position
  private int targetPosition = startingPosition;
  private int lastExecutedPosition;
  private final static int onTargetThreshold = 100;

  private int manualIncrement = 25;

  // define soft limits
  private int upPositionLimit = 5200;
  private int downPositionLimit = 0;

  // PID slots
  public final static int slot_up = 0;
  public final static int slot_down = 1;

  // Gains
  Gains upGains = new Gains(5, 0, 0, 0, 150, 1);
  Gains downGains = new Gains(5, 0, 0, 0, 150, 1);

  // Motion Parameters 
  public int maxVelocityUp = 400;
  public int maxAccelerationUp = 400;
  public int maxVelocityDown = 400;
  public int maxAccelerationDown = 400;

  // motors
  public WPI_TalonSRX motorMaster = new WPI_TalonSRX(RobotMap.liftMasterPort);
  public WPI_TalonSRX motorSlave = new WPI_TalonSRX(RobotMap.liftSlavePort);

  // limit switch
  public DigitalInput limitSwitch;

  public Lift(boolean tunable) {

    // pass argument from robot.java
    this.tunable = tunable;

    // reset talons
    motorMaster.configFactoryDefault();
    motorSlave.configFactoryDefault();

    //Set up encoder
    motorMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);	
    motorMaster.setSensorPhase(false);
    motorMaster.setInverted(false);
    motorMaster.setNeutralMode(NeutralMode.Coast);

    motorSlave.setInverted(true);
    motorSlave.setNeutralMode(NeutralMode.Coast);

    motorSlave.follow(motorMaster);

    motorMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, RobotMap.timeoutMs);
    motorMaster.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10, RobotMap.timeoutMs);
		motorMaster.configNeutralDeadband(RobotMap.neutralDeadband, RobotMap.timeoutMs);
		motorSlave.configNeutralDeadband(RobotMap.neutralDeadband, RobotMap.timeoutMs);
		motorMaster.configPeakOutputForward(+1.0, RobotMap.timeoutMs);
		motorMaster.configPeakOutputReverse(-1.0, RobotMap.timeoutMs);
		motorSlave.configPeakOutputForward(+1.0, RobotMap.timeoutMs);
    motorSlave.configPeakOutputReverse(-1.0, RobotMap.timeoutMs);
    		
		// set speeds
		motorMaster.configMotionAcceleration(maxAccelerationUp, RobotMap.timeoutMs);
		motorMaster.configMotionCruiseVelocity(maxVelocityUp, RobotMap.timeoutMs);

    // setup a PIDF slot for going up
		motorMaster.config_kP(slot_up, upGains.kP, RobotMap.timeoutMs);
		motorMaster.config_kI(slot_up, upGains.kI, RobotMap.timeoutMs);
		motorMaster.config_kD(slot_up, upGains.kD, RobotMap.timeoutMs);
		motorMaster.config_kF(slot_up, upGains.kF, RobotMap.timeoutMs);
		motorMaster.config_IntegralZone(slot_up, upGains.kIZone, RobotMap.timeoutMs);
    motorMaster.configClosedLoopPeakOutput(slot_up, upGains.kPeakOutput, RobotMap.timeoutMs);

    // setup a PIDF slot for going down
    motorMaster.config_kP(slot_down, downGains.kP, RobotMap.timeoutMs);
		motorMaster.config_kI(slot_down, downGains.kI, RobotMap.timeoutMs);
		motorMaster.config_kD(slot_down, downGains.kD, RobotMap.timeoutMs);
		motorMaster.config_kF(slot_down, downGains.kF, RobotMap.timeoutMs);
		motorMaster.config_IntegralZone(slot_down, downGains.kIZone, RobotMap.timeoutMs);
    motorMaster.configClosedLoopPeakOutput(slot_down, downGains.kPeakOutput, RobotMap.timeoutMs);
			
		/**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
    motorMaster.configClosedLoopPeriod(slot_up, closedLoopTimeMs, RobotMap.timeoutMs);
    motorMaster.configClosedLoopPeriod(slot_down, closedLoopTimeMs, RobotMap.timeoutMs);

    // set profile slot, this will be updated when gains need to change
    motorMaster.selectProfileSlot(slot_up, RobotMap.pidPrimary);

    // set encoder and set point to starting position
    resetPosition();
    setTargetPosition(getStartingPosition());

    limitSwitch = new DigitalInput(RobotMap.liftLimitSwitchPort);
  }

  // LIMIT SWITCH
  public boolean limitTriggered() {
    return limitSwitch.get();
  }

  // ENCODER
  public void resetPosition() {
    motorMaster.setSelectedSensorPosition(getStartingPosition(), 0, 10);
  }
  
  public int getPosition() {
    int pos = motorMaster.getSensorCollection().getQuadraturePosition();
    return pos;    
  }

  // get velocity
  public double getCurrentVelocity() {
    return motorMaster.getSelectedSensorVelocity();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new LiftMM());
  }



  
  // MANUAL MOVEMENT CODE WITH PercentOutput
  // set Manual command as default command to lift manually
  // use to tune feedForward
  public void raise() {
    motorMaster.set(ControlMode.PercentOutput, .8);
  }
  public void lower() {
    if(limitTriggered()) {
      stop();
    } else {
      motorMaster.set(ControlMode.PercentOutput, .4);  
    }
  }
  public void move(double move) {
    motorMaster.set(ControlMode.PercentOutput, move);
  }
  public void stop() {
    motorMaster.set(ControlMode.PercentOutput, 0);
  }


  

  // MOTION MAGIC CONTROL
  // set MM as default command to manualControlStick with Motion Magic
  // uses targetPostion to move to a setpoint
  public void motionMagicControl() {
    
    // print a random number to visually show when MM restarts
    if(tunable)
      SmartDashboard.putNumber("Motion Control Starting", Math.random());

    // setup gain slot, velocity and acceleration
    manageMotion(targetPosition);

    // Percent to add to Closed Loop Output
    double feedForward = getFeedForward();

    // Do It!!!
		motorMaster.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, feedForward);
    motorSlave.follow(motorMaster);

    // keep track so we know when targetPosition has changed (in periodic method)
    lastExecutedPosition = targetPosition;
  }

  // setup gain slot, velocity and acceleration
  public void manageMotion(int targetPosition) {
    int currentPosition = getPosition();

    // going up
    if(currentPosition < targetPosition) {

      // set accel and velocity for going up
      motorMaster.configMotionAcceleration(maxAccelerationUp, RobotMap.timeoutMs);
      motorMaster.configMotionCruiseVelocity(maxVelocityUp, RobotMap.timeoutMs);

      motorMaster.selectProfileSlot(slot_up, RobotMap.pidPrimary);

    // going down
    } else {

      // set accel and velocity for going down
      motorMaster.configMotionAcceleration(maxAccelerationDown, RobotMap.timeoutMs);
      motorMaster.configMotionCruiseVelocity(maxVelocityDown, RobotMap.timeoutMs);

      // select the down gains
      motorMaster.selectProfileSlot(slot_down, RobotMap.pidPrimary);
    }
    
  }
  
  // get the position variables
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

  // get/set/increment the target position
  public int getTargetPosition() {
    return this.targetPosition;
  }

  public boolean setTargetPosition(int targetPosition) {
    if(isPositionValid(targetPosition)) {
      this.targetPosition = targetPosition;

      SmartDashboard.putNumber("Lift Target", targetPosition);

      return true;
    } else {
      return false;
    }
    
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

   // return the angle of the arm based on the current encoder value
   public double getAngle() {
    int currentPosition = getPosition();

    // divide the encoder position when arm is horizontal by the angle displacement
    // so if you moved the arm 30 degrees and read 1000 ticks, it would be 1000/30 ticks per degree
    // subtract horizontalAngleDisplacement to make the horizontal postion 0 degrees
    double ticksPerDegree = horizontalPosition/horizontalAngleDisplacement;
    double angle = currentPosition / ticksPerDegree - horizontalAngleDisplacement;
    return angle;
  }

  // take the horizontalHoldOutput and linearize it by multiplying it by the cos of the angle of the arm
  public double getFeedForward() {
    double radians = Math.toRadians(getAngle());
    double feedForward = horizontalHoldOutput * Math.cos(radians);

    if(getAngle() < -55)
      feedForward = 0;

    return feedForward;
  }

  // to tune, call this in the initialize method of a command
  public void initializer() {
    if(tunable) {
      SmartDashboard.putNumber("Lift Target", getStartingPosition());
      SmartDashboard.putNumber("Lift Position", this.getPosition());
      SmartDashboard.putNumber("Lift Velocity", this.getCurrentVelocity());
      SmartDashboard.putNumber("Lift Closed Loop Error", this.motorMaster.getClosedLoopError());

      SmartDashboard.putNumber("Lift Velocity Up", maxVelocityUp);
      SmartDashboard.putNumber("Lift Velocity Down", maxVelocityDown);
      SmartDashboard.putNumber("Lift Acceleration Up", maxAccelerationUp);
      SmartDashboard.putNumber("Lift Acceleration Down", maxAccelerationDown);

      SmartDashboard.putNumber("Lift Up kP", upGains.kP);
      SmartDashboard.putNumber("Lift Down kP", downGains.kP);

      SmartDashboard.putNumber("Lift Manual Increment", manualIncrement);

      SmartDashboard.putNumber("Lift Angle", getAngle());
      SmartDashboard.putNumber("Lift Gravity Compensation", Math.cos(Math.toRadians(getAngle())));
      SmartDashboard.putNumber("Lift FeedForward", getFeedForward());
    }
  }

  // to tune, call this in the execute method of a command
  public void periodicLoop() {
    SmartDashboard.putNumber("Lift Position", this.getPosition());

    SmartDashboard.putBoolean("Limit Test",  limitTriggered());

    // if limit switch is triggered, reset to starting position
    if(limitTriggered()) {
      resetPosition();
    }

    boolean changed = false;

    // if targetPosition has changed since MM was last called, call MM again
    if(lastExecutedPosition != targetPosition)
      changed = true;

    // check for manual control
    // only increment if joystick is plugged into the laptop
    double manualControlStick = Robot.oi.stick.getPOV();
    if(Robot.oi.joystickIsPluggedIn() && manualControlStick == 0) {
      incrementTargetPosition(manualIncrement);
      changed = true;
    } else if (manualControlStick == 180) {
      incrementTargetPosition(-manualIncrement);
      changed = true;
    }

    // tune from Smart Dashboard
    if(tunable) {
      SmartDashboard.putNumber("Lift Velocity", this.getCurrentVelocity());
      SmartDashboard.putNumber("Lift Angle", getAngle());
      SmartDashboard.putNumber("Lift Gravity Compensation", Math.cos(Math.toRadians(getAngle())));
      SmartDashboard.putNumber("Lift FeedForward", getFeedForward());

      int sdManualIncrement = (int) SmartDashboard.getNumber("Lift Manual Increment", manualIncrement);
      if(sdManualIncrement != manualIncrement) 
        manualIncrement = sdManualIncrement;

      int sdVelUp = (int) SmartDashboard.getNumber("Lift Velocity Up", maxVelocityUp);
      if(sdVelUp != maxVelocityUp) 
        this.maxVelocityUp = sdVelUp;

      int sdVelDown = (int) SmartDashboard.getNumber("Lift Velocity Down", maxVelocityDown);
      if(sdVelDown != maxVelocityDown) 
        this.maxVelocityDown = sdVelDown;

      int sdAccelUp = (int) SmartDashboard.getNumber("Lift Acceleration Up", maxAccelerationUp);
      if(sdAccelUp != maxAccelerationUp)
        this.maxAccelerationUp = sdAccelUp;

      int sdAccelDown = (int) SmartDashboard.getNumber("Lift Accerleration Down", maxAccelerationDown);
      if(sdAccelDown != maxAccelerationDown)
        this.maxAccelerationDown = sdAccelDown;

      double sdUpP = SmartDashboard.getNumber("Lift Up kP", upGains.kP);
      if(sdUpP != upGains.kP) {
        upGains.kP = sdUpP;
        motorMaster.config_kP(slot_up, upGains.kP, RobotMap.timeoutMs);
      }

      double sdDownP = SmartDashboard.getNumber("Lift Down kP", downGains.kP);
      if(sdDownP != downGains.kP) {
        downGains.kP = sdDownP;
        motorMaster.config_kP(slot_down, downGains.kP, RobotMap.timeoutMs);
      }
      
      int sdTargetPosition = (int) SmartDashboard.getNumber("Lift Target", targetPosition);
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