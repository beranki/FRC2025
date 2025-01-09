package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Meter;

// Third party Hardware Imports
import com.studica.frc.AHRS;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

//Java Imports
import java.io.File;

public class DriveFSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		TELEOP_STATE
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private final SwerveDrive swerveDrive;

	private int lockedSpeakerId;

	/* ======================== Constructor ======================== */
	/**
	 * Create DriveFSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 * @param directory The directory where the configuration files are located.
	 */
	public DriveFSMSystem(File directory) {
		// Perform hardware init
		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

		try {
			swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, 0, 0);

			// setting customizations for swerve drive
			swerveDrive.setHeadingCorrection(true);
			swerveDrive.setCosineCompensator(true);
			// swerveDrive.setAngularVelocityCompensation(true,
			// 											true,
			// 											Constants.ANGLE_VELO_COEFF);
			swerveDrive.setModuleEncoderAutoSynchronize(false,
														1);
			swerveDrive.pushOffsetsToEncoders();

			swerveDrive.stopOdometryThread(); // need to update odometry manually on main thread.

		} catch (Exception e) {
			throw new RuntimeException(e);
		}

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public FSMState getCurrentState() {
		return currentState;
	}
	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = FSMState.TELEOP_STATE;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {

		swerveDrive.updateOdometry(); //update odometry manually every input loop

		switch (currentState) {
			case TELEOP_STATE:
				handleTeleOpState(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);
	}

	/**
	 * Performs specific action based on the autoState passed in.
	 * @param autoState autoState that the subsystem executes.
	 * @return if the action carried out in this state has finished executing
	 */
	public boolean updateAutonomous(AutoFSMState autoState) {
		switch (autoState) {
			case STATE1:
				return handleAutoState1();
			case STATE2:
				return handleAutoState2();
			case STATE3:
				return handleAutoState3();
			default:
				return true;
		}
	}

	/* ======================== Private methods ======================== */
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private FSMState nextState(TeleopInput input) {
		switch (currentState) {
			case TELEOP_STATE:
				return FSMState.TELEOP_STATE;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in TELEOP_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleTeleOpState(TeleopInput input) {

		if (input == null) {
			return;
		}

		double translationVal = MathUtil.applyDeadband(
			input.getDriveLeftJoystickY(), OperatorConstants.LEFT_Y_DEADBAND)
			* swerveDrive.getMaximumChassisVelocity();
		double strafeVal = MathUtil.applyDeadband(
			input.getDriveLeftJoystickX(), OperatorConstants.LEFT_X_DEADBAND)
			* swerveDrive.getMaximumChassisVelocity();
		double rotationVal = MathUtil.applyDeadband(
			input.getDriveRightJoystickX(), OperatorConstants.RIGHT_X_DEADBAND)
			* swerveDrive.getMaximumChassisAngularVelocity();

		swerveDrive.drive(
			new Translation2d(-translationVal, -strafeVal),
			-rotationVal,
			true,
			false
		);
	}

	/**
	 * Resets the odometry to the specified pose.
	 * @param pose The pose to reset the odometry to.
	 */
	public void resetOdometry(Pose2d pose) {
		swerveDrive.resetOdometry(pose);
	}

	/**
	 * Get the current pose of the robot.
	 * @return The current pose as a Pose2d object.
	 */
	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

	/**
	 * Zero the gyro of the swerve drive.
	 */
	public void zeroGyro() {
		swerveDrive.zeroGyro();
	}

	/**
	 * Set the motor brake mode.
	 * @param brake True to enable brake mode, false to disable.
	 */
	public void setMotorBrake(boolean brake) {
		swerveDrive.setMotorIdleMode(brake);
	}

	/**
	 * Get the current heading of the robot.
	 * @return The current heading as a Rotation2d object.
	 */
	public Rotation2d getHeading() {
		return getPose().getRotation();
	}

	/**
	 * Get the pitch of the robot.
	 * @return The current pitch as a Rotation2d object.
	 */
	public Rotation2d getPitch() {
		return swerveDrive.getPitch();
	}

	private boolean handleAutoState1() {
		return true;
	}

	private boolean handleAutoState2() {
		return true;
	}

	private boolean handleAutoState3() {
		return true;
	}
}
