package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;

// CTRE Imports

// Third party Hardware Imports
import com.kauailabs.navx.frc.AHRS;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.constants.SwerveConstants.OIConstants;
import frc.robot.constants.SwerveConstants.DriveConstants;
import frc.robot.HardwareMap;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;
import frc.robot.SwerveModule;

public class DriveFSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		TELEOP_STATE,
		ALIGN_TAG_STATE
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	public SwerveDrivePoseEstimator swerveDrivePoseEstimator;
	public SwerveModule frModule;
	public SwerveModule flModule;
	public SwerveModule brModule;
	public SwerveModule blModule;
	
	private AHRS gyro = new AHRS(SPI.Port.kMXP);

	/* ======================== Constructor ======================== */
	/**
	 * Create DriveFSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DriveFSMSystem(String cameraName) {
		// Perform hardware init
		gyro.reset();
		gyro.setAngleAdjustment(0);

		flModule = new SwerveModule(
			HardwareMap.FRONT_LEFT_DRIVING_CAN_ID,
			HardwareMap.FRONT_LEFT_TURNING_CAN_ID,
			DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET
		);

		frModule = new SwerveModule(
			HardwareMap.FRONT_RIGHT_DRIVING_CAN_ID,
			HardwareMap.FRONT_RIGHT_TURNING_CAN_ID,
			DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET
		);

		blModule = new SwerveModule(
			HardwareMap.REAR_LEFT_DRIVING_CAN_ID,
			HardwareMap.REAR_LEFT_TURNING_CAN_ID,
			DriveConstants.REAR_LEFT_CHASSIS_ANGULAR_OFFSET
		);

		brModule = new SwerveModule(
			HardwareMap.REAR_RIGHT_DRIVING_CAN_ID,
			HardwareMap.REAR_RIGHT_TURNING_CAN_ID,
			DriveConstants.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET
		);

		swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
			DriveConstants.DRIVE_KINEMATICS,
			getHeading(),
			getModulePositions(),
			new Pose2d()
		);

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
		swerveDrivePoseEstimator.update(
			getHeading(), getModulePositions()
		);

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
		return true;
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
		
		double translationVal = MathUtil.applyDeadband(
			input.getDriveLeftJoystickY(), OIConstants.DRIVE_DEADBAND);
		double strafeVal = MathUtil.applyDeadband(
			input.getDriveLeftJoystickX(), OIConstants.DRIVE_DEADBAND
		);
		double rotationVal = MathUtil.applyDeadband(
			input.getDriveRightJoystickX(), OIConstants.DRIVE_DEADBAND
		);
		
		drive(
			new Translation2d(translationVal, strafeVal)
				.times(DriveConstants.MAX_SPEED_METERS_PER_SECOND),
			rotationVal * DriveConstants.MAX_ANGULAR_SPEED,
			true
		);
	}

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        ChassisSpeeds speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

		if (fieldRelative) {
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
		}

		SwerveModuleState[] swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);

        flModule.setDesiredState(swerveModuleStates[0]);
        frModule.setDesiredState(swerveModuleStates[1]);
        blModule.setDesiredState(swerveModuleStates[2]);
        brModule.setDesiredState(swerveModuleStates[(2 + 1)]);

	}

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);

        flModule.setDesiredState(desiredStates[0]);
        frModule.setDesiredState(desiredStates[1]);
        blModule.setDesiredState(desiredStates[2]);
        brModule.setDesiredState(desiredStates[(2 + 1)]);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        
		states[0] = flModule.getState();
		states[1] = frModule.getState();
		states[2] = blModule.getState();
		states[(2 + 1)] = brModule.getState();

        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

		positions[0] = flModule.getPosition();
		positions[1] = frModule.getPosition();
		positions[2] = blModule.getPosition();
		positions[(2 + 1)] = brModule.getPosition();

        return positions;
    }

    public Pose2d getPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        swerveDrivePoseEstimator.resetPosition(
			getHeading(), getModulePositions(), pose
		);
    }

	public void resetOdometry(Pose2d pose) {
		swerveDrivePoseEstimator.resetPosition(
			Rotation2d.fromDegrees(getHeading().getDegrees()),
			new SwerveModulePosition[] {
				flModule.getPosition(),
				frModule.getPosition(),
				blModule.getPosition(),
				brModule.getPosition()
			},
			pose);
	  }

	public void resetEncoders() {
		flModule.resetEncoders();
		blModule.resetEncoders();
		frModule.resetEncoders();
		brModule.resetEncoders();
	}

	public Rotation2d getHeading() {
        return gyro.getRotation2d();
    }

    public void setHeading(Rotation2d heading) {
        setPose(new Pose2d(getPose().getTranslation(), heading));
    }

	/* Used to reset the swerve drive pose based on the estimated vision measurements */
	public void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds) {
        swerveDrivePoseEstimator.addVisionMeasurement(visionRobotPose, timeStampSeconds);
    }

    public void zeroHeading() {
        setHeading(new Rotation2d());
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
