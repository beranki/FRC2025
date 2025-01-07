package frc.robot;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
	// ID numbers for devices on the CAN bus
	public static final int TALON_ID_DRIVE_FRONT_RIGHT = 1;
	public static final int TALON_ID_TURN_FRONT_RIGHT = 2;

	public static final int TALON_ID_DRIVE_BACK_RIGHT = 3;
	public static final int TALON_ID_TURN_BACK_RIGHT = 4;

	public static final int TALON_ID_DRIVE_FRONT_LEFT = 5;
	public static final int TALON_ID_TURN_FRONT_LEFT = 6;

	public static final int TALON_ID_DRIVE_BACK_LEFT = 7;
	public static final int TALON_ID_TURN_BACK_LEFT = 8;

	public static final int CANCODER_ID_FRONT_LEFT = 1;
	public static final int CANCODER_ID_FRONT_RIGHT = 1;
	public static final int CANCODER_ID_BACK_LEFT = 1;
	public static final int CANCODER_ID_BACK_RIGHT = 1;

	public static final int PIGEON_GYRO_ID = 1;

	public static final int CAN_ID_SPARK_SHOOTER = 0;

	/* ===== Hardware Availability ===== */
	/**
	 * Check if drive hardware is available to the RoboRIO.
	 * @return true if drive hardware is present
	 */
	public static boolean isDriveHardwarePresent() {
		return true;
	}

	/**
	 * Check if mech 1 hardware is available to the RoboRIO.
	 * @return true if mech hardware is present
	 */
	public static boolean isMech1HardwarePresent() {
		return false;
	}

	/**
	 * Check if mech 2 hardware is available to the RoboRIO.
	 * @return true if mech 2 hardware is present
	 */
	public static boolean isMech2HardwarePresent() {
		return false;
	}

	/**
	 * Check if drive hardware is available to the RoboRIO.
	 * @return true if drive hardware is present
	 */
	public static boolean isCVHardwarePresent() {
		return false;
	}
}
