package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.PS4Controller;

/**
 * Common class for providing driver inputs during Teleop.
 *
 * This class is the sole owner of WPILib input objects and is responsible for
 * polling input values. Systems may query TeleopInput via its getter methods
 * for inputs by value, but may not access the internal input objects.
 */
public class TeleopInput {
	/* ======================== Constants ======================== */
	private static final int DRIVE_CONTROLLER_PORT = 0;
	private static final int MECH_CONTROLLER_PORT = 1;

	/* ======================== Private variables ======================== */
	// Input objects
	private PS4Controller mechController;
	private PS4Controller driverController;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		driverController = new PS4Controller(DRIVE_CONTROLLER_PORT);
		mechController = new PS4Controller(MECH_CONTROLLER_PORT);
	}

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
	// Method names should be descriptive of the behavior, so the
	// control mapping is hidden from other classes.

	/* ------------------------ Drive Controller ------------------------ */
	
	/**
	 * Get X axis of Right Joystick.
	 * @return Axis value
	 */
	public double getDriveRightJoystickX() {
		return driverController.getRightX();
	}

	/**
	 * Get Y axis of Right Joystick.
	 * @return Axis value
	 */
	public double getDriveRightJoystickY() {
		return driverController.getRightY();
	}

	/**
	 * Get Y axis of Left Joystick.
	 * @return Axis value
	 */
	public double getDriveLeftJoystickY() {
		return driverController.getLeftY();
	}

	/**
	 * Get Y axis of Left Joystick.
	 * @return Axis value
	 */
	public double getDriveLeftJoystickX() {
		return driverController.getLeftX();
	}

	/**
	 * Get if triangle button of Right Joystick is pressed.
	 * @return If Triangle button is pressed
	 */
	public boolean isAlignmentButtonPressed() {
		return driverController.getTriangleButton();
	}

	/* ------------------------ Mech Controller ------------------------ */
	
	/**
	 * Get X axis of Left Joystick.
	 * @return Axis value
	 */
	public double getMechLeftJoystickX() {
		return mechController.getLeftX();
	}
	/**
	 * Get Y axis of Left Joystick.
	 * @return Axis value
	 */
	public double getMechLeftJoystickY() {
		return mechController.getLeftY();
	}
	/**
	 * Get the value of the shooter button.
	 * @return True if button is pressed
	 */
	public boolean isShooterButtonPressed() {
		return mechController.getTriangleButton();
	}
	/**
	 * Get the value of the intake button.
	 * @return True if button is pressed
	 */
	public boolean isIntakeButtonPressed() {
		return mechController.getCircleButton();
	}

	/* ======================== Private methods ======================== */

}
