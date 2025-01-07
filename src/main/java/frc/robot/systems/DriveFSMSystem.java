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
import edu.wpi.first.math.util.Units;

// CTRE Imports
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.configs.Pigeon2Configuration;

// Third party Hardware Imports
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.constants.Constants.VisionConstants;
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
	
	public Pigeon2 gyro;
	private PhotonCamera camera;

	private int lockedSpeakerId;

	/* ======================== Constructor ======================== */
	/**
	 * Create DriveFSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DriveFSMSystem(String cameraName) {
		// Perform hardware init
		gyro = new Pigeon2(HardwareMap.PIGEON_GYRO_ID);
		gyro.getConfigurator().apply(new Pigeon2Configuration());
		gyro.setYaw(0);

		lockedSpeakerId = 0;

		flModule = new SwerveModule(
			HardwareMap.TALON_ID_DRIVE_FRONT_LEFT,
			HardwareMap.TALON_ID_TURN_FRONT_LEFT,
			HardwareMap.CANCODER_ID_FRONT_LEFT, 
			Constants.Swerve.FRONT_LEFT_ANGLE_OFFSET
		);

		frModule = new SwerveModule(
			HardwareMap.TALON_ID_DRIVE_FRONT_RIGHT,
			HardwareMap.TALON_ID_TURN_FRONT_RIGHT,
			HardwareMap.CANCODER_ID_FRONT_RIGHT, 
			Constants.Swerve.FRONT_RIGHT_ANGLE_OFFSET
		);

		blModule = new SwerveModule(
			HardwareMap.TALON_ID_DRIVE_BACK_LEFT,
			HardwareMap.TALON_ID_TURN_BACK_LEFT,
			HardwareMap.CANCODER_ID_BACK_LEFT, 
			Constants.Swerve.BACK_LEFT_ANGLE_OFFSET
		);

		brModule = new SwerveModule(
			HardwareMap.TALON_ID_DRIVE_BACK_RIGHT,
			HardwareMap.TALON_ID_TURN_BACK_RIGHT,
			HardwareMap.CANCODER_ID_BACK_RIGHT, 
			Constants.Swerve.BACK_RIGHT_ANGLE_OFFSET
		);

		swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
			Constants.Swerve.SWERVE_DRIVE_KINEMATICS,
			getGyroYaw(),
			getModulePositions(),
			new Pose2d()
		);

		camera = new PhotonCamera(cameraName);

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
			getGyroYaw(), getModulePositions()
		);

		switch (currentState) {
			case TELEOP_STATE:
				handleTeleOpState(input);
				break;
			case ALIGN_TAG_STATE:
				handleAlignTagState(input, lockedSpeakerId);
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
				if (input.isAlignmentButtonPressed()) {
					return FSMState.ALIGN_TAG_STATE;
				} else {
					return FSMState.TELEOP_STATE;
				}
			case ALIGN_TAG_STATE:
				if (input.isAlignmentButtonPressed()) {
					return FSMState.ALIGN_TAG_STATE;
				} else {
					return FSMState.TELEOP_STATE;
				}
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
				.times(Constants.Swerve.MAX_SPEED_METERS),
			rotationVal * Constants.Swerve.MAX_ANGULAR_VELOCITY,
			true,
			true
		);
	}

	/**
	 * Handle behavior in ALIGN_TAG_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleAlignTagState(TeleopInput input, int lockedSpeakerId) {
        boolean targetVisible = false;
        double targetYaw = 0.0;
        double targetRange = 0.0;

        var results = camera.getAllUnreadResults();

        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == lockedSpeakerId) {

                        targetYaw = target.getYaw();
                        targetRange =
                                PhotonUtils.calculateDistanceToTargetMeters(
                                        VisionConstants.CAMERA_HEIGHT_METERS, // Measured with a tape measure, or in CAD.
                                        VisionConstants.TARGET_HEIGHT_METERS, // From 2024 game manual for ID 7
                                        Units.degreesToRadians(VisionConstants.CAMERA_PITCH_DEGREES), // Measured with a protractor, or in CAD.
                                        Units.degreesToRadians(target.getPitch()));

                        targetVisible = true;
                    }
                }
            }
        }

        if (targetVisible) {
            double rotationVal =
                    (VisionConstants.VISION_DES_ANGLE_DEGREES - targetYaw)
						* VisionConstants.VISION_TURN_P;
            double translationVal =
                    (VisionConstants.VISION_DES_RANGE_METERS - targetRange)
						* VisionConstants.VISION_STRAFE_P;
		
			drive(
				new Translation2d(translationVal, 0)
					.times(Constants.Swerve.MAX_SPEED_METERS),
				rotationVal * Constants.Swerve.MAX_ANGULAR_VELOCITY,
				false,
				true
			);
		}
	}

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

		if (fieldRelative) {
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
		}

		SwerveModuleState[] swerveModuleStates = Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED_METERS);

        flModule.setDesiredState(swerveModuleStates[0], isOpenLoop);
        frModule.setDesiredState(swerveModuleStates[1], isOpenLoop);
        blModule.setDesiredState(swerveModuleStates[2], isOpenLoop);
        brModule.setDesiredState(swerveModuleStates[(2 + 1)], isOpenLoop);

	}

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED_METERS);

        flModule.setDesiredState(desiredStates[0], false);
        frModule.setDesiredState(desiredStates[1], false);
        blModule.setDesiredState(desiredStates[2], false);
        brModule.setDesiredState(desiredStates[(2 + 1)], false);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        
		states[0] = flModule.getState();
		states[1] = frModule.getState();
		states[2] = blModule.getState();
		states[(2 + 1)] = brModule.getState();

        return states;
    }

    public ChassisSpeeds getMeasuredSpeeds() {
        return Constants.Swerve.SWERVE_DRIVE_KINEMATICS
			.toChassisSpeeds(getModuleStates());
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
			getGyroYaw(), getModulePositions(), pose
		);
    }

    public Rotation2d getGyroYaw() {
        return new Rotation2d(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute() {
        flModule.resetToAbsolute();
        frModule.resetToAbsolute();
        blModule.resetToAbsolute();
        brModule.resetToAbsolute();        
    }

	public Rotation2d getHeading() {
        return getPose().getRotation();
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
