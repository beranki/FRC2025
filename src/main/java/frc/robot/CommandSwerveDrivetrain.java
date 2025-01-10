package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.DriveConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
@SuppressWarnings("rawtypes")
public class CommandSwerveDrivetrain extends SwerveDrivetrain {

	/* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
	private static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
	/* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
	private static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;
	/* Keep track if we've ever applied the operator perspective before or not */
	private boolean hasAppliedOperatorPerspective = false;

	private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds =
		new SwerveRequest.ApplyRobotSpeeds();

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

		setupPathplanner();
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
	 * Sets up the Pathplanner configuration for the swerve drivetrain.
	 */
	public void setupPathplanner() {
		double driveBaseRadius = 0;
		for (var moduleLocation : getModuleLocations()) {
			driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
		}

		try {
			var config = RobotConfig.fromGUISettings();

			AutoBuilder.configure(
				() -> getState().Pose,
				this::resetPose,
				() -> getState().Speeds,
				(speeds, feedforwards) -> setControl(
					pathApplyRobotSpeeds.withSpeeds(speeds)
					.withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
					.withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
				),
				new PPHolonomicDriveController(
					DriveConstants.AUTO_TRANSLATION_PID,
					DriveConstants.AUTO_ROTATION_PID
				),
					config,
				() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
			);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	// @Override
	// public void simulationPeriodic() {
	// 	/* Assume 20ms update rate, get battery voltage from WPILib */
	// 	updateSimState(0.02, RobotController.getBatteryVoltage());
	// }
}
