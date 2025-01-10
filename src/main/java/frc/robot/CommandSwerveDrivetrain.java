package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain {

	/* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
	private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.k180deg;
	/* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
	private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.kZero;
	/* Keep track if we've ever applied the operator perspective before or not */
	private boolean m_hasAppliedOperatorPerspective = false;

	public CommandSwerveDrivetrain(
		SwerveDrivetrainConstants drivetrainConstants,
		SwerveModuleConstants<?, ?, ?>... modules
	) {
		super(
			TalonFX::new, TalonFX::new, CANcoder::new,
			drivetrainConstants, modules
		);
	}

	public void applyOperatorPerspective() {
		if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
			DriverStation.getAlliance().ifPresent(allianceColor -> {
				setOperatorPerspectiveForward(
					allianceColor == Alliance.Red
						? kRedAlliancePerspectiveRotation
						: kBlueAlliancePerspectiveRotation
				);
				m_hasAppliedOperatorPerspective = true;
			});
		}
	}

	public void setupPathplanner() {
		double driveBaseRadius = 0;
		for (var moduleLocation : getModuleLocations()) {
			driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
		}

		AutoBuilder.configureHolonomic(
			()->this.getState().Pose, // Supplier of current robot pose
			this::seedFieldRelative,  // Consumer for seeding pose against auto
			this::getCurrentRobotChassisSpeeds,
			(speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
			new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
											new PIDConstants(10, 0, 0),
											TunerConstants.kSpeedAt12VoltsMps,
											driveBaseRadius,
											new ReplanningConfig()),
			() -> {
					// Boolean supplier that controls when the path will be mirrored for the red alliance
					// This will flip the path being followed to the red side of the field during auto only.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red & !DriverStation.isTeleop();
					}
					return false;
			},
			this); // Subsystem for requirements
		}
	}

	// @Override
	// public void simulationPeriodic() {
	//     /* Assume 20ms update rate, get battery voltage from WPILib */
	//     updateSimState(0.02, RobotController.getBatteryVoltage());
	// }
}