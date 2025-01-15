package frc.robot.utils.simulation;

// CTRE Imports
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.Pair;
// WPI Imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

// Measures
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;

import static edu.wpi.first.units.Units.Feet;
// Units
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

// Maple-Sim
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;


// Local Imports
import frc.robot.constants.SimConstants;
import frc.robot.Robot;
import static frc.robot.utils.simulation.MotorSimUtil.TalonFXMotorControllerSim;
import static frc.robot.utils.simulation.MotorSimUtil.TalonFXMotorControllerWithRemoteCanCoderSim;;


/**
 * <h2>Injects Maple-Sim simulation data into a CTRE swerve drivetrain.</h2>
 *
 * <p>This class retrieves simulation data from Maple-Sim and injects it
 * into the CTRE {@link com.ctre.phoenix6.swerve.SwerveDrivetrain} instance.</p>
 *
 * <p>It replaces the {@link com.ctre.phoenix6.swerve.SimSwerveDrivetrain} class.</p>
 */
public class MapleSimSwerveDrivetrain {
	private final Pigeon2SimState pigeonSim;
	private final SimSwerveModule[] simModules;
	private final SwerveDriveSimulation mapleSimDrive;

	/**
	 * <h2>Constructs a drivetrain simulation using the specified parameters.</h2>
	 *
	 * @param simPeriod the time period of the simulation
	 * @param robotMassWithBumpers the total mass of the robot, including bumpers
	 * @param bumperLengths pair of the lengths of bumpers, first X second Y
	 * (influences the collision space of the robot)
	 * @param moduleMotors pair of the motors in the modules, first drive second steer
	 * typically <code>DCMotor.getKrakenX60()</code>
	 * @param wheelCOF the coefficient of friction of the drive wheels
	 * @param moduleLocations the locations of the swerve modules on the robot,
	 * in the order <code>FL, FR, BL, BR</code>
	 * @param pigeon the {@link Pigeon2} IMU used in the drivetrain
	 * @param modules the {@link SwerveModule}s, typically obtained via
	 * {@link SwerveDrivetrain#getModules()}
	 * @param moduleConstants the constants for the swerve modules
	 */
	@SuppressWarnings("unchecked")
	public MapleSimSwerveDrivetrain(
			Time simPeriod,
			Mass robotMassWithBumpers,
			Pair<Distance, Distance> bumperLengths,
			Pair<DCMotor, DCMotor> moduleMotors,
			double wheelCOF,
			Translation2d[] moduleLocations,
			Pigeon2 pigeon,
			SwerveModule<TalonFX, TalonFX, CANcoder>[] modules,
			SwerveModuleConstants<
				TalonFXConfiguration,
				TalonFXConfiguration,
				CANcoderConfiguration>... moduleConstants) {
		this.pigeonSim = pigeon.getSimState();
		simModules = new SimSwerveModule[moduleConstants.length];

		var simulationConfig = DriveTrainSimulationConfig.Default()
				.withRobotMass(robotMassWithBumpers)
				.withBumperSize(bumperLengths.getFirst(), bumperLengths.getSecond())
				.withGyro(COTS.ofPigeon2())
				.withCustomModuleTranslations(moduleLocations)
				.withSwerveModule(new SwerveModuleSimulationConfig(
						moduleMotors.getFirst(),
						moduleMotors.getSecond(),
						moduleConstants[0].DriveMotorGearRatio,
						moduleConstants[0].SteerMotorGearRatio,
						Volts.of(moduleConstants[0].DriveFrictionVoltage),
						Volts.of(moduleConstants[0].SteerFrictionVoltage),
						Meters.of(moduleConstants[0].WheelRadius),
						KilogramSquareMeters.of(moduleConstants[0].SteerInertia),
						wheelCOF));

		mapleSimDrive = new SwerveDriveSimulation(
			simulationConfig,
			new Pose2d(
				new Translation2d(
					Feet.of(SimConstants.STARTING_POS_X_FT),
					Feet.of(SimConstants.STARTING_POS_Y_FT)
				),
				new Rotation2d()
			)
		);

		SwerveModuleSimulation[] moduleSimulations = mapleSimDrive.getModules();

		for (int i = 0; i < this.simModules.length; i++) {
			simModules[i] = new SimSwerveModule(
				moduleConstants[0], moduleSimulations[i], modules[i]);
		}

		SimulatedArena.overrideSimulationTimings(simPeriod, 1);
		SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
	}

	/**
	 * Gets the SwerveModuleSimulation injected into the SwerveDrivetrain.
	 * @return the swerveModuleSimulation of this drivetrain
	 */
	public SwerveDriveSimulation getMapleSimDrive() {
		return mapleSimDrive;
	}

	/**
	 * <h2>Update the simulation.</h2>
	 *
	 * <p>Updates the Maple-Sim simulation and injects the results into the simulated CTRE devices,
	 * including motors and the IMU.</p>
	 */
	public void update() {
		SimulatedArena.getInstance().simulationPeriodic();
		pigeonSim.setRawYaw(
				mapleSimDrive.getSimulatedDriveTrainPose().getRotation().getMeasure());
		pigeonSim.setAngularVelocityZ(RadiansPerSecond.of(
				mapleSimDrive
					.getDriveTrainSimulatedChassisSpeedsRobotRelative().omegaRadiansPerSecond));
	}

	/**
	 * <h1>Represents the simulation of a single {@link SwerveModule}.</h1>
	 */
	protected static class SimSwerveModule {
		private final SwerveModuleConstants<
			TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> moduleConstant;
		private final SwerveModuleSimulation moduleSimulation;

		public SimSwerveModule(
				SwerveModuleConstants<
					TalonFXConfiguration,
					TalonFXConfiguration,
					CANcoderConfiguration> moduleConstants,
				SwerveModuleSimulation swerveModuleSimulation,
				SwerveModule<TalonFX, TalonFX, CANcoder> module) {
			this.moduleConstant = moduleConstants;
			this.moduleSimulation = swerveModuleSimulation;
			moduleSimulation.useDriveMotorController(
				new TalonFXMotorControllerSim(module.getDriveMotor()));
			moduleSimulation.useSteerMotorController(
					new TalonFXMotorControllerWithRemoteCanCoderSim(
						module.getSteerMotor(), module.getEncoder()));
		}

		public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration,
			CANcoderConfiguration> getModuleConstants() {
			return moduleConstant;
		}
	}

	/**
	 * <h2>Regulates all {@link SwerveModuleConstants} for a drivetrain simulation.</h2>
	 *
	 * <p>This method processes an array of {@link SwerveModuleConstants}
	 * to apply necessary adjustments
	 * for simulation purposes, ensuring compatibility and avoiding known bugs.</p>
	 * @param moduleConstants unregulated moduleConstants for each swerve module
	 * @return regulated swerve module constants as per sim requirements
	 * @see #regulateModuleConstantForSimulation(SwerveModuleConstants)
	 */
	public static SwerveModuleConstants<?, ?, ?>[]
		regulateModuleConstantsForSimulation(
			SwerveModuleConstants<?, ?, ?>[] moduleConstants) {
		for (SwerveModuleConstants<?, ?, ?> moduleConstant : moduleConstants) {
			regulateModuleConstantForSimulation(moduleConstant);
		}

		return moduleConstants;
	}

	/**
	 * <h2>Regulates the {@link SwerveModuleConstants} for a single module.</h2>
	 *
	 * This method applies specific adjustments to the {@link SwerveModuleConstants} for simulation
	 * purposes. These changes have no effect on real robot operations and address
	 * known simulation bugs:
	 *
	 * <ul>
	 *     <li><strong>Inverted Drive Motors:</strong>
	 * 			Prevents drive PID issues caused by inverted configurations.</li>
	 *     <li><strong>Non-zero CanCoder Offsets:</strong>
	 * 			Fixes potential module state optimization issues.</li>
	 *     <li><strong>Steer Motor PID:</strong>
	 * 			Adjusts PID values tuned for real robots to improve simulation performance.</li>
	 * </ul>
	 *
	 * <h4>Note:This function is skipped when running on a real robot,
	 * 		ensuring no impact on constants used on real robot hardware.</h4>
	 * @param moduleConstants unregulated module constants that
	 * 					      are used on the real robot.
	 */
	private static void regulateModuleConstantForSimulation(
		SwerveModuleConstants<?, ?, ?> moduleConstants) {
		// Skip regulation if running on a real robot
		if (Robot.isReal()) {
			return;
		}

		// Apply simulation-specific adjustments to module constants
		moduleConstants
				// Disable encoder offsets
				.withEncoderOffset(0)
				// Disable motor inversions for drive and steer motors
				.withDriveMotorInverted(false)
				.withSteerMotorInverted(false)
				// Disable CanCoder inversion
				.withEncoderInverted(false)
				// Adjust steer motor PID gains for simulation
				.withSteerMotorGains(moduleConstants.SteerMotorGains
						.withKP(SimConstants.MODULE_STEER_P)  // Proportional gain
						.withKD(SimConstants.MODULE_STEER_D)) // Derivative gain
				// Adjust friction voltages
				.withDriveFrictionVoltage(Volts.of(SimConstants.DRIVE_FRICTION_VOLTS))
				.withSteerFrictionVoltage(Volts.of(SimConstants.STEER_FRICTION_VOLTS))
				// Adjust steer inertia
				.withSteerInertia(KilogramSquareMeters.of(SimConstants.STEER_INERTIA_KGMS2));
	}
}
