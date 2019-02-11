package frc.robot.util;


import java.nio.ByteBuffer;

import frc.robot.RobotMap;

/**
 * Class that contains various math functions.
 */
public class Maths {

	// DISTANCE CONVERSIONS

	public static double centimetersToInches(double val) {
		return val / 2.54;
	}

	public static double inchesToCentimeters(double val) {
		return val * 0.393700787;
	}
	
	public static double inchesToFeet(double inches) {
		return inches / 12;
	}

	public static double feetToMeters(double val) {
		return 100 * inchesToCentimeters(val / 12);
	}

	public static double metersToFeet(double val) {
		return 1 / (100 * inchesToCentimeters(val / 12));
	}


	// TIME CONVERSIONS

	public static double secondsToMinutes(double val) {
		return val / 60;
	}

	public static double minutesToSeconds(double val) {
		return val * 60;
	}

	public static double minutesToHours(double val) {
		return val / 60;
	}

	public static double hoursToMinutes(double val) {
		return val * 60;
	}

	
	// ENCODER CONVERSIONS

	public static double ticksToMeters(double ticks) {
		return inchesToCentimeters(ticksToInches(ticks)) * 100;
	}

	public static double feetToEncoderTicks(double feet) {
		return inchesToEncoderTicks(feet * 12);
	}

	public static double inchesToEncoderTicks(double inches) {
		return inches / (Math.PI * RobotMap.wheelDiameter) * RobotMap.sensorUnitsPerRotation;
	}

	public static double ticksToInches(double ticks) {
		return ticks / RobotMap.sensorUnitsPerRotation * (Math.PI * RobotMap.wheelDiameter);
	}

	
	// TICKS PER TENTH AND RPM/RPS CONVERSIONS

	public static double ticksPerTenthToRevsPerMinute(double ticksPerTenthSecond) {
		return ticksPerTenthSecond /RobotMap.sensorUnitsPerRotation * 600;
	}

	public static double revsPerSecondToTicksPerTenth(double revsPerSecond) {
        return revsPerSecond * RobotMap.sensorUnitsPerRotation / 10;
    }

	public static double revsPerMinuteToTicksPerTenth(double revsPerMinute) {
        return revsPerMinute * RobotMap.sensorUnitsPerRotation / 600;
    }

	public static double revsPerMinuteToMetersPerSecond(double revsPerMinute) {
		return revsPerMinute * feetToMeters(RobotMap.wheelDiameter * Math.PI / 12) / 60;
	}

	public static double ticksPerTenthToMetersPerSecond(double ticksPerTenth) {
		return revsPerMinuteToMetersPerSecond(ticksPerTenthToRevsPerMinute(ticksPerTenth));
	}
	

	// FEED FORWARD

	public static double calculateFeedForward(double rpm) {
		final double MAX_MOTOR_OUTPUT = 1023;
		final double NATIVE_UNITS_PER_100 = rpm / 600 * RobotMap.sensorUnitsPerRotation;
		return MAX_MOTOR_OUTPUT/NATIVE_UNITS_PER_100;
	}


	// BYTE STUFF - UNTESTED

	public static String bytesToHex(byte[] bytes) {
		char[] hexChars = new char[bytes.length * 2];
		for (int j = 0; j < bytes.length; j++) {
			int v = bytes[j] & 0xFF;
			hexChars[j * 2] = HEX_ARRAY[v >>> 4];
			hexChars[j * 2 + 1] = HEX_ARRAY[v & 0x0F];
		}
		return new String(hexChars);
	}
	public static String intsToHex(int[] ints) {
		StringBuilder hexString = new StringBuilder();
		for (int i : ints)
			hexString.append(Integer.toHexString(i));

		return hexString.toString();
	}
	public static String bbToString(ByteBuffer bb) {
		final byte[] b = new byte[bb.remaining()];
		bb.duplicate().get(b);
		bb.rewind();
		return Maths.bytesToHex(b);
	}

	private final static char[] HEX_ARRAY = "0123456789ABCDEF".toCharArray();

	/**
	 * Clamps a value between a minimum and maximum, inclusive.
	 * 
	 * @param val the value to clamp
	 * @param min the minimum value
	 * @param max the maximum value
	 * @return {@code val}, if {@code val} is between [{@code min}, {@code max}]
	 * 		   {@code min}, if {@code val} is <= {@code min}
	 * 		   {@code min}, if {@code val} is >= {@code min}
	 */
	public static double clamp(double val, double min, double max) {
		if (val <= min)
			val = min;
		else if (val >= max)
			val = max;
		
		return val;
	}
	/**
	 * Interpolates a value between a minimum and maximum value via a percentage
	 *
	 * @param percent percent to use to interpolate between a and b
	 * @param a some value
	 * @param b some other value
	 * @return a value between a and b given a percent, with 0 being min, and 1 being max
	 */
	public static double lerp(double percent, double a, double b) {
		percent = clamp(percent, 0, 1);

		return percent * b - percent * a + a;
	}
}