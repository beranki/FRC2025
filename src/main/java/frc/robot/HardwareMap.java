package frc.robot;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
	// ID numbers for devices on the CAN bus
	public static final int CAN_ID_ELEVATOR = 14; // FINAL
	public static final int CAN_ID_CLIMBER = 8; // FINAL

	// playing with fusion CAN chain
	public static final int FUNNEL_TOF_ID = 13; // TBD

	// rio - dio ports
	public static final int ELEVATOR_GROUND_LIMIT_SWITCH_DIO_PORT = 0; // FINAL
	public static final int ELEVATOR_TOP_LIMIT_SWITCH_DIO_PORT = 1; // FINAL

	public static final int FUNNEL_BREAK_BEAM_DIO_PORT = 2; // FINAL

	public static final int CLIMBER_LIMIT_SWITCH_DIO_PORT = 3; // FINAL

	// rio - pwm ports
	public static final int FUNNEL_SERVO_PWM_PORT = 0; // FINAL
	public static final int LED_STRIP_PWM_PORT = 1; // FINAL

	public static final int ELASTIC_WEBSERVER_PORT = 5800;

	/* ===== Hardware Availability ===== */
	/**
	 * Check if drive hardware is available to the RoboRIO.
	 * @return true if drive hardware is present
	 */
	public static boolean isDriveHardwarePresent() {
		return true;
	}

	/**
	 * Check if elevator hardware is available to the RoboRIO.
	 * Constructor requires funnel hardware to be available as well.
	 * @return true if elevator hardware is present
	 */
	public static boolean isElevatorHardwarePresent() {
		return true;
	}

	/**
	 * Check if climber hardware is available to the RoboRIO.
	 * @return true if climber hardware is present
	 */
	public static boolean isClimberHardwarePresent() {
		return false;
	}

	/**
	 * Check if funnel hardware is available to the RoboRIO.
	 * @return true if funnel hardware is present
	 */
	public static boolean isFunnelHardwarePresent() {
		return true;
	}

	/**
	 * Check if CV hardware is available to the RoboRIO.
	 * @return true if drive hardware is present
	 */
	public static boolean isCVHardwarePresent() {
		return true;
	}

	/**
	 * Check if LED hardware is available to the RoboRIO.
	 * @return true if LED hardware is present
	 */
	public static boolean isLEDPresent() {
		return true;
	}
}
