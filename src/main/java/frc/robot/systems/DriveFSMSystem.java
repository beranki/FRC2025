package frc.robot.systems;

import edu.wpi.first.math.MathUtil;
// WPILib Imports
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

//CTRE Imports
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.SwerveLogging;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;
import frc.robot.CommandSwerveDrivetrain;

public class DriveFSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		TELEOP_STATE,
	}

	@SuppressWarnings("unused")
	private static final float MOTOR_RUN_POWER = 0.1f;
	@SuppressWarnings("unused")
	private final SlewRateLimiter xLimiter = new SlewRateLimiter(2);
	@SuppressWarnings("unused")
	private final SlewRateLimiter yLimiter = new SlewRateLimiter(0.5);
	@SuppressWarnings("unused")
	private final SlewRateLimiter rotLimiter = new SlewRateLimiter(0.5);
	private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
		// kSpeedAt12Volts desired top speed
	private final double maxAngularRate =
		RotationsPerSecond.of(DriveConstants.MAC_ANGULAR_VELO_RPS).in(RadiansPerSecond);
		//3/4 rps angle velo

	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
		.withDeadband(maxSpeed * DriveConstants.DRIVE_DEADBAND) // 20% deadband
		.withRotationalDeadband(maxAngularRate * DriveConstants.ROTATION_DEADBAND) //10% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop for drive motors
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	private final SwerveLogging logger = new SwerveLogging(maxSpeed);
	private CommandSwerveDrivetrain drivetrain;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	/* ======================== Constructor ======================== */
	/**
	 * Create DriveFSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DriveFSMSystem() {
		// Perform hardware init
		drivetrain = TunerConstants.createDrivetrain();

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
		if (input == null) {
			return;
		}

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
				if (input != null) {
					return FSMState.TELEOP_STATE;
				}

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleTeleOpState(TeleopInput input) {
		logger.applyStateLogging(drivetrain.getState());
		drivetrain.applyOperatorPerspective();

		drivetrain.setControl(
			drive.withVelocityX(-MathUtil.applyDeadband(
				input.getDriveLeftJoystickY(), DriveConstants.DRIVE_DEADBAND
				) * maxSpeed) // Drive forward with negative Y (forward)
			.withVelocityY(
				-MathUtil.applyDeadband(
					input.getDriveLeftJoystickX(), DriveConstants.DRIVE_DEADBAND
					) * maxSpeed) // Drive left with negative X (left)
			.withRotationalRate(
				-MathUtil.applyDeadband(
					input.getDriveRightJoystickX(), DriveConstants.DRIVE_DEADBAND
					) * maxAngularRate) // Drive counterclockwise with negative X (left)
		);

		if (input.getDriveTriangleButton()) {
			drivetrain.setControl(brake);
		}

		if (input.getDriveCircleButton()) {
			drivetrain.setControl(
				point.withModuleDirection(new Rotation2d(-input.getDriveLeftJoystickY(),
					-input.getDriveLeftJoystickX()))
			);
		}

		if (input.getDriveBackButtonPressed()) {
			drivetrain.seedFieldCentric();
		}
	}

	/**
	 * Performs action for auto STATE1.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoState1() {
		return true;
	}

	/**
	 * Performs action for auto STATE2.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoState2() {
		return true;
	}

	/**
	 * Performs action for auto STATE3.
	 * @return if the action carried out has finished executing
	 */
	private boolean handleAutoState3() {
		return true;
	}
}
