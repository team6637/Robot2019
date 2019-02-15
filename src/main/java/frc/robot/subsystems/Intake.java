
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class Intake extends Subsystem {

  public boolean tunable = false;

  public DoubleSolenoid solenoid = new DoubleSolenoid(RobotMap.intakeSolenoidLow, RobotMap.intakeSolenoidHigh);
  public VictorSPX motor = new VictorSPX(RobotMap.intakeMotorPort);

  public double intakeSpeed = .8;

  public Intake(boolean tunable) {
    this.tunable = tunable;
  }

  public void bringIt() {
    motor.set(ControlMode.PercentOutput, intakeSpeed);
  }

  public void sendIt() {
    motor.set(ControlMode.PercentOutput, -intakeSpeed);
  }

  public void hatchRetract() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void hatchEject() {
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void initDefaultCommand() {
  }
}