
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
import frc.robot.commands.WristMM;
import frc.robot.subsystems.Lift.Load;
import frc.robot.util.Gains;


public class Wrist extends Subsystem {
  
  public boolean tunable;

  // how far does the encoder have to travel to get from starting position to horizontal
  private int horizontalOffset = 700;

  // how many degrees has the arm traveled to get to the horizontal position from start position
  private double horizontalAngleDisplacement = 68;

  // what is the least output required to hold the arm up in the horizontal position
  private double  horizontalHoldOutput = 0.125;

  // setup predefined setpoints
  private int startingPosition = 2000;
  private int cargoIntakePosition = 488;
  private int cargoBayPosition = 1000;
  private int rocketCargo1Position = 1135;
  private int rocketCargo2Position = 690;
  private int rocketCargo3Position = 515;
  private int rocketHatch1Position = 1800;
  private int rocketHatch2Position = 1250;
  private int rocketHatch3Position = 360;

  private int targetPosition = startingPosition;
  private int lastExecutedPosition;
  private final static int onTargetThreshold = 100;

  private int manualIncrement = 15;

  // define soft limits
  private int upPositionLimit = 2000;
  private int downPositionLimit = 0;

  // PID slots
  public final static int slot_up_empty = 0;
  public final static int slot_up_cargo = 1;
  public final static int slot_up_hatch = 2;
  public final static int slot_down = 3;

  // Gains
  Gains upEmptyGains = new Gains(2.5, 0, 0, 0, 150, 1);
  Gains upCargoGains = new Gains(2.5, 0, 0, 0, 150, 1);
  Gains upHatchGains = new Gains(2.5, 0, 0, 0, 150, 1);
  Gains downGains = new Gains(2.5, 0, 0, 0, 150, 1);

  // Motion Parameters 
  public int maxVelocityUp = 600;
  public int maxAccelerationUp = 600;
  public int maxVelocityDown = 600;
  public int maxAccelerationDown = 600;

  public WPI_TalonSRX motor = new WPI_TalonSRX(RobotMap.wristMotorPort);

  // limit switch
  public DigitalInput limitSwitch;

  public Wrist(boolean tunable) {
    this.tunable = tunable;

    // reset talons
    motor.configFactoryDefault();

    motor.setInverted(false);
    motor.setNeutralMode(NeutralMode.Coast);

    //Set up encoder
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    motor.setSensorPhase(false);
    
    motor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, RobotMap.timeoutMs);
    motor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10, RobotMap.timeoutMs);
    motor.configNeutralDeadband(RobotMap.neutralDeadband, RobotMap.timeoutMs);
    motor.configPeakOutputForward(+1.0, RobotMap.timeoutMs);
    motor.configPeakOutputReverse(-1.0, RobotMap.timeoutMs);
    
    // set speeds
    motor.configMotionAcceleration(maxAccelerationUp, RobotMap.timeoutMs);
    motor.configMotionCruiseVelocity(maxVelocityUp, RobotMap.timeoutMs);
    
    // setup a PIDF slot for going up empty
    motor.config_kP(slot_up_empty, upEmptyGains.kP, RobotMap.timeoutMs);
    motor.config_kI(slot_up_empty, upEmptyGains.kI, RobotMap.timeoutMs);
    motor.config_kD(slot_up_empty, upEmptyGains.kD, RobotMap.timeoutMs);
    motor.config_kF(slot_up_empty, upEmptyGains.kF, RobotMap.timeoutMs);
    motor.config_IntegralZone(slot_up_empty, upEmptyGains.kIZone, RobotMap.timeoutMs);
    motor.configClosedLoopPeakOutput(slot_up_empty, upEmptyGains.kPeakOutput, RobotMap.timeoutMs);
    
    // setup a PIDF slot for going up with cargo
    motor.config_kP(slot_up_cargo, upCargoGains.kP, RobotMap.timeoutMs);
    motor.config_kI(slot_up_cargo, upCargoGains.kI, RobotMap.timeoutMs);
    motor.config_kD(slot_up_cargo, upCargoGains.kD, RobotMap.timeoutMs);
    motor.config_kF(slot_up_cargo, upCargoGains.kF, RobotMap.timeoutMs);
    motor.config_IntegralZone(slot_up_cargo, upCargoGains.kIZone, RobotMap.timeoutMs);
    motor.configClosedLoopPeakOutput(slot_up_cargo, upCargoGains.kPeakOutput, RobotMap.timeoutMs);
    
    // setup a PIDF slot for going up with the hatch
    motor.config_kP(slot_up_hatch, upHatchGains.kP, RobotMap.timeoutMs);
    motor.config_kI(slot_up_hatch, upHatchGains.kI, RobotMap.timeoutMs);
    motor.config_kD(slot_up_hatch, upHatchGains.kD, RobotMap.timeoutMs);
    motor.config_kF(slot_up_hatch, upHatchGains.kF, RobotMap.timeoutMs);
    motor.config_IntegralZone(slot_up_hatch, upHatchGains.kIZone, RobotMap.timeoutMs);
    motor.configClosedLoopPeakOutput(slot_up_hatch, upHatchGains.kPeakOutput, RobotMap.timeoutMs);
    
    // setup a PIDF slot for going down
    motor.config_kP(slot_down, downGains.kP, RobotMap.timeoutMs);
    motor.config_kI(slot_down, downGains.kI, RobotMap.timeoutMs);
    motor.config_kD(slot_down, downGains.kD, RobotMap.timeoutMs);
    motor.config_kF(slot_down, downGains.kF, RobotMap.timeoutMs);
    motor.config_IntegralZone(slot_down, downGains.kIZone, RobotMap.timeoutMs);
    motor.configClosedLoopPeakOutput(slot_down, downGains.kPeakOutput, RobotMap.timeoutMs);
    
    int closedLoopTimeMs = 1;
    motor.configClosedLoopPeriod(slot_up_empty, closedLoopTimeMs, RobotMap.timeoutMs);
    motor.configClosedLoopPeriod(slot_up_cargo, closedLoopTimeMs, RobotMap.timeoutMs);
    motor.configClosedLoopPeriod(slot_up_hatch, closedLoopTimeMs, RobotMap.timeoutMs);
    motor.configClosedLoopPeriod(slot_down, closedLoopTimeMs, RobotMap.timeoutMs);
    
    // set profile slot, this will be updated when gains need to change
    motor.selectProfileSlot(slot_up_empty, RobotMap.pidPrimary);
    resetPosition();

    limitSwitch = new DigitalInput(RobotMap.wristLimitSwitchPort);
  }

  // LIMIT SWITCH
  public boolean limitTriggered() {
    return limitSwitch.get();
  }

  public int getPosition() {
    return motor.getSelectedSensorPosition();
  }

  public void resetPosition() {
    motor.setSelectedSensorPosition(2000, 0, 10);
  }

  // return the angle of the arm based on the current encoder value
  public double getAngle() {
    int currentPosition = getPosition();

    // divide the encoder position when arm is horizontal by the angle displacement
    // so if you moved the arm 30 degrees and read 1000 ticks, it would be 1000/30 ticks per degree
    // subtract horizontalAngleDisplacement to make the horizontal postion 0 degrees
    double ticksPerDegree = horizontalOffset/horizontalAngleDisplacement;
    double relativeAngle = (currentPosition - horizontalOffset) / ticksPerDegree;
    double absoluteAngle = relativeAngle + Robot.liftSubsystem.getAngle();

    if(tunable) {
      SmartDashboard.putNumber("Wrist Relative Angle", relativeAngle);
      SmartDashboard.putNumber("Wrist Absolute Angle", absoluteAngle);
    }
      
    return absoluteAngle;
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
    manageMotion(targetPosition);

    // Percent to add to Closed Loop Output
    //double feedForward = getFeedForward();
    double feedForward = 0;

    // Do It!!!
		motor.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, feedForward);

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

      // check if we are loaded, set the pid gains accordingly
      Load currentLoad = Robot.liftSubsystem.currentLoad;
      switch(currentLoad) {
        case EMPTY :
          motor.selectProfileSlot(slot_up_empty, RobotMap.pidPrimary);
          break;
        case CARGO :
          motor.selectProfileSlot(slot_up_cargo, RobotMap.pidPrimary);
          break;
        case HATCH :
          motor.selectProfileSlot(slot_up_hatch, RobotMap.pidPrimary);
          break;
      }

    } else {
      
      // set accel and velocity for going down
      motor.configMotionAcceleration(maxAccelerationDown, RobotMap.timeoutMs);
      motor.configMotionCruiseVelocity(maxVelocityDown, RobotMap.timeoutMs);

      // select the down gains
      motor.selectProfileSlot(slot_down, RobotMap.pidPrimary);
    }

  }

  public int getstartingPosition() {
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

      if(tunable)
        SmartDashboard.putNumber("Wrist Target", targetPosition);

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

  // take the horizontalHoldOutput and linearize it by multiplying it by the cos of the angle of the arm
  public double getFeedForward() {
    double radians = Math.toRadians(getAngle());
    double feedForward = horizontalHoldOutput * Math.cos(radians);

    if(tunable) {
      SmartDashboard.putNumber("Wrist FeedForward", feedForward);
      SmartDashboard.putNumber("Wrist Radians", radians);
      SmartDashboard.putNumber("Wrist Cos", Math.cos(radians));
    }
    return feedForward;
  }

  public void initializer() {

    if(tunable) {
      SmartDashboard.putNumber("Wrist Target", targetPosition);
      SmartDashboard.putNumber("Wrist Position", this.getPosition());
      SmartDashboard.putNumber("Wrist Velocity", this.getCurrentVelocity());
      SmartDashboard.putNumber("Wrist Closed Loop Error", this.motor.getClosedLoopError());

      SmartDashboard.putNumber("Wrist Velocity Up", maxVelocityUp);
      SmartDashboard.putNumber("Wrist Velocity Down", maxVelocityDown);
      SmartDashboard.putNumber("Wrist Acceleration Up", maxAccelerationUp);
      SmartDashboard.putNumber("Wrist Acceleration Down", maxAccelerationDown);

      SmartDashboard.putNumber("Wrist Up Empty kP", upEmptyGains.kP);

      SmartDashboard.putNumber("Wrist Up Cargo kP", upCargoGains.kP);
      SmartDashboard.putNumber("Wrist Up Hatch kP", upHatchGains.kP);
      SmartDashboard.putNumber("Wrist Down kP", downGains.kP);

      SmartDashboard.putNumber("Wrist Manual Increment", manualIncrement);
    }
  }

  // to tune, call this in the execute method of a command
  public void periodicLoop() {
    boolean changed = false;

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

      SmartDashboard.putNumber("Wrist Position", this.getPosition());

      SmartDashboard.putBoolean("Limit Test",  limitTriggered());

      // if limit switch is triggered, reset to starting position
      if(limitTriggered()) {
        resetPosition();
      }    
  
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

      double sdUpEmptyP = SmartDashboard.getNumber("Wrist Up Empty kP", upEmptyGains.kP);
      if(sdUpEmptyP != upEmptyGains.kP) {
        upEmptyGains.kP = sdUpEmptyP;
        motor.config_kP(slot_up_empty, upEmptyGains.kP, RobotMap.timeoutMs);
        changed = true;
      }

      double sdUpCargoP = SmartDashboard.getNumber("Wrist Up Cargo kP", upCargoGains.kP);
      if(sdUpCargoP != upCargoGains.kP) {
        upCargoGains.kP = sdUpCargoP;
        motor.config_kP(slot_up_cargo, upCargoGains.kP, RobotMap.timeoutMs);
      }

      double sdUpHatchP = SmartDashboard.getNumber("Wrist Up Hatch kP", upHatchGains.kP);
      if(sdUpHatchP != upHatchGains.kP) {
        upHatchGains.kP = sdUpHatchP;
        motor.config_kP(slot_up_hatch, upHatchGains.kP, RobotMap.timeoutMs);
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
