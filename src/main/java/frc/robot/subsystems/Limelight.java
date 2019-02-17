package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.LimelightDriverCam;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Limelight extends Subsystem {

  public boolean tunable = false;
  public NetworkTable table;
  public int ledMode, camMode;

  public Limelight(boolean tunable) {
    this.tunable = tunable;

    table = NetworkTableInstance.getDefault().getTable("limelight");

	setPipeline(1);
	setLedMode(1);
	setCameraMode(1);
  }

	/**
	 * Gets whether a target is detected by the Limelight.
	 * 
	 * @return true if a target is detected, false otherwise.
	 */
	public boolean isTarget() {
		return getValue("tv").getDouble(0.0) == 1;
	}

	/**
	 * Horizontal offset from crosshair to target (-27 degrees to 27 degrees).
	 * 
	 * @return tx as reported by the Limelight.
	 */
	public double getTx() {
		return getValue("tx").getDouble(0.0);
	}

	/**
	 * Vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees).
	 * 
	 * @return ty as reported by the Limelight.
	 */
	public double getTy() {
		return getValue("ty").getDouble(0.0);
	}

	/**
	 * Area that the detected target takes up in total camera FOV (0% to 100%).
	 * 
	 * @return Area of target.
	 */
	public double getTa() {
		return getValue("ta").getDouble(0.0);
	}

	/**
	 * Gets target skew or rotation (-90 degrees to 0 degrees).
	 * 
	 * @return Target skew.
	 */
	public double getTs() {
		return getValue("ts").getDouble(0.0);
	}

	/**
	 * Sets LED mode of Limelight.
	 * 
	 * @param mode
	 *            Light mode for Limelight.
	 */
	public void setLedMode(int ledMode) {
		getValue("ledMode").setNumber(ledMode);
	}

	/**
	 * Sets camera mode for Limelight.
	 * 
	 * @param mode
	 *            Camera mode for Limelight.
	 */
	public void setCameraMode(int cameraMode) {
		getValue("camMode").setNumber(cameraMode);
	}

	/**
	 * Sets pipeline number (0-9 value).
	 * 
	 * @param number
	 *            Pipeline number (0-9).
	 */
	public void setPipeline(int number) {
		getValue("pipeline").setNumber(number);
	}

	/**
	 * Helper method to get an entry from the Limelight NetworkTable.
	 * 
	 * @param key
	 *            Key for entry.
	 * @return NetworkTableEntry of given entry.
	 */
	private NetworkTableEntry getValue(String key) {
		return table.getEntry(key);
	}

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new LimelightDriverCam());
  }
}
