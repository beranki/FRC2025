package frc.robot;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
	// ID numbers for devices on the CAN bus
	public static final int FRONT_LEFT_DRIVING_CAN_ID = 24; //28
	public static final int FRONT_RIGHT_DRIVING_CAN_ID = 26; //32
	public static final int REAR_LEFT_DRIVING_CAN_ID = 32; //26
	public static final int REAR_RIGHT_DRIVING_CAN_ID = 28; //24

	public static final int FRONT_LEFT_TURNING_CAN_ID = 23; //27
	public static final int FRONT_RIGHT_TURNING_CAN_ID = 25; //31
	public static final int REAR_LEFT_TURNING_CAN_ID = 31; //25
	public static final int REAR_RIGHT_TURNING_CAN_ID = 27; //23

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
