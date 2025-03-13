package frc.robot;


import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import choreo.trajectory.SwerveSample;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.TunerConstants;
import frc.robot.simulation.MapleSimSwerveDrivetrain;
import frc.robot.simulation.SimSwerveDrivetrainConfig;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */

public class CommandSwerveDrivetrain extends
	SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {

	/* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
	private static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
	/* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
	private static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;
	/* Keep track if we've ever applied the operator perspective before or not */
	private boolean hasAppliedOperatorPerspective = false;

	private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds =
		new SwerveRequest.ApplyRobotSpeeds();

	private final SwerveRequest.ApplyFieldSpeeds pathApplyFieldSpeeds =
		new SwerveRequest.ApplyFieldSpeeds();

	private final PIDController autoXPid = new PIDController(17.5, 0, 0);
	private final PIDController autoYPid = new PIDController(17.5, 0, 0);
	private final PIDController autoHeadingPid = new PIDController(0.8, 0, 0);

	private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain;
	private Notifier simNotifier;
	/**
	 * Constructs a CommandSwerveDrivetrain with the specified drivetrain constants and modules.
	 *
	 * @param drivetrainConstants the constants for the swerve drivetrain
	 * @param modules the swerve modules
	 */
	public CommandSwerveDrivetrain(
		SwerveDrivetrainConstants drivetrainConstants,
		SwerveModuleConstants<?, ?, ?>... modules
	) {
		super(
			TalonFX::new, TalonFX::new, CANcoder::new,
			drivetrainConstants,
			MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules)
		);

		if (Robot.isSimulation()) {
			setupSimulation(
				new Pose2d(0, 0, new Rotation2d())
			);
		}

		autoXPid.setTolerance(1e-3);
		autoYPid.setTolerance(1e-3);
		// setupPathplanner();
	}

	/**
	 * Applies the operator perspective based on the alliance color.
	 * Blue alliance sees forward as 0 degrees (toward red alliance wall).
	 * Red alliance sees forward as 180 degrees (toward blue alliance wall).
	 */
	public void applyOperatorPerspective() {
		if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
			DriverStation.getAlliance().ifPresent(allianceColor -> {
				setOperatorPerspectiveForward(
					allianceColor == Alliance.Red
						? RED_ALLIANCE_PERSPECTIVE_ROTATION
						: BLUE_ALLIANCE_PERSPECTIVE_ROTATION
				);
				hasAppliedOperatorPerspective = true;
			});
		}
	}

	/**
	 * Follows the given trajectory sample by adjusting the chassis speeds and heading.
	 *
	 * @param sample the swerve sample containing the desired trajectory information
	 */
	public void followTrajectory(SwerveSample sample) {
		autoHeadingPid.enableContinuousInput(-Math.PI, Math.PI);

		Pose2d pose = getState().Pose;

		ChassisSpeeds targspeeds = sample.getChassisSpeeds();
		targspeeds.vxMetersPerSecond += autoXPid.calculate(pose.getX(), sample.x);
		targspeeds.vyMetersPerSecond += autoYPid.calculate(pose.getY(), sample.y);
		targspeeds.omegaRadiansPerSecond += autoHeadingPid.calculate(
			pose.getRotation().getRadians(), sample.heading);

		setControl(
			pathApplyFieldSpeeds.withSpeeds(targspeeds)
			.withWheelForceFeedforwardsX(sample.moduleForcesX())
			.withWheelForceFeedforwardsY(sample.moduleForcesY())
		);
	}

	/**
	 * Get the SwerveModulePosition for localized odo.
	 * @return swerve module position
	 */
	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] pos = new SwerveModulePosition[(2 + 2)];

		for (int i = 0; i < (2 + 2); i++) {
			pos[i] = getModule(i).getPosition(false);
		}

		return pos;
	}

	private void setupSimulation(Pose2d startingPose) {
		mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(
			SimSwerveDrivetrainConfig.getDefault()
				.withModuleLocations(getModuleLocations())
				.withPigeon(getPigeon2())
				.withModules(getModules())
				.withModuleConstants(
						TunerConstants.FRONT_LEFT,
						TunerConstants.FRONT_RIGHT,
						TunerConstants.BACK_LEFT,
						TunerConstants.BACK_RIGHT
				)
				.withStartingPose(startingPose)
			);
	}

	/**
	 * Get the sim drive train as a MapleSimSwerveDrivetrain.
	 * @return the sim drivetrain of the current class, or null if not simulation
	 */
	public MapleSimSwerveDrivetrain getSimDrivetrain() {
		return mapleSimSwerveDrivetrain;
	}

	/**
	 * Return the pose of the drivetrain.
	 * @return pose
	 */
	public Pose2d getPose() {
		if (Robot.isSimulation()) {
			return getSimDrivetrain().getDriveSimulation().getSimulatedDriveTrainPose();
		} else {
			return getState().Pose;
		}
	}

	/**
	 * Return the chassis speeds of the drivetrain relative to the robot.
	 * @return chassis speeds relative to robot
	 */
	public ChassisSpeeds getRobotRelativeChassisSpeeds() {
		if (Robot.isSimulation()) {
			return getSimDrivetrain().getDriveSimulation()
				.getDriveTrainSimulatedChassisSpeedsRobotRelative();
		} else {
			return getState().Speeds;
		}
	}

	/**
	 * Get the chassis speeds of the drivetrain relative to the field.
	 * @return chassis speeds relative to field
	 */
	public ChassisSpeeds getFieldRelativeChassisSpeeds() {
		if (Robot.isSimulation()) {
			return getSimDrivetrain().getDriveSimulation()
				.getDriveTrainSimulatedChassisSpeedsFieldRelative();
		} else {
			// Converts robot relative speeds to field relative
			// TODO: Determine if getState().Speeds is robot relative or field relative
			return ChassisSpeeds.fromRobotRelativeSpeeds(
				getState().Speeds,
				getPigeon2().getRotation2d()
			);
		}
	}

}
