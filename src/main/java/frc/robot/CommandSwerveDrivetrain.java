package frc.robot;

import com.ctre.phoenix6.Utils;
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
import edu.wpi.first.wpilibj.RobotController;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
@SuppressWarnings("rawtypes")
public class CommandSwerveDrivetrain extends SwerveDrivetrain {
	private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
	private Notifier simNotifier = null;
	private double lastSimTime;


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

	private final PIDController autoXPid = new PIDController(5, 0, 0);
	private final PIDController autoYPid = new PIDController(5, 0, 0);
	private final PIDController autoHeadingPid = new PIDController(0.75, 0, 0);

	/**
	 * Constructs a CommandSwerveDrivetrain with the specified drivetrain constants and modules.
	 *
	 * @param drivetrainConstants the constants for the swerve drivetrain
	 * @param modules the swerve modules
	 */
	@SuppressWarnings("unchecked")
	public CommandSwerveDrivetrain(
		SwerveDrivetrainConstants drivetrainConstants,
		SwerveModuleConstants<?, ?, ?>... modules
	) {
		super(
			TalonFX::new, TalonFX::new, CANcoder::new,
			drivetrainConstants, modules
		);

		if (Utils.isSimulation()) {
			startSimThread();
		}
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
	 * @return module positions
	 */
	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] pos = new SwerveModulePosition[(2 + 2)];

		for (int i = 0; i < (2 + 2); i++) {
			pos[i] = getModule(i).getPosition(false);
		}

		return pos;
	}

	/**
	 * Update the simulation state.
	 */
	private void startSimThread() {
		lastSimTime = Utils.getCurrentTimeSeconds();

		/* Run simulation at a faster rate so PID gains behave more reasonably */
		simNotifier = new Notifier(() -> {
			final double currentTime = Utils.getCurrentTimeSeconds();
			double deltaTime = currentTime - lastSimTime;
			lastSimTime = currentTime;

			/* use the measured time delta, get battery voltage from WPILib */
			updateSimState(deltaTime, RobotController.getBatteryVoltage());
		});
		simNotifier.startPeriodic(SIM_LOOP_PERIOD);
	}
}
