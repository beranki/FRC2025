package frc.robot.utils.simulation;

// CTRE Imports
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;

// WPI Imports
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

// Measures
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;

// Units
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Pounds;

// Maple-Sim
import frc.robot.constants.RobotConstants;

// Local Imports
import frc.robot.constants.SimConstants;

/**
 * Configuration helper for the MapleSimSwerveDrivetrain class.
 */
public class SimSwerveDrivetrainConfig {
	private Mass robotMass;
	private Time simPeriod;
	private Distance bumperWidth;
	private Distance bumperLength;
	private DCMotor moduleDriveMotor;
	private DCMotor moduleSteerMotor;
	private double wheelCOF;
	private Translation2d[] moduleLocations;
	private Pigeon2 pigeon = new Pigeon2(0);
	private SwerveModule<TalonFX, TalonFX, CANcoder>[] modules;

	/**
	 * Get the default configuration of a MapleSimSwerveDrivetrain.
	 * @return the default configuration
	 */
	public static SimSwerveDrivetrainConfig getDefault() {
		return new SimSwerveDrivetrainConfig()
			.withRobotMass(Pounds.of(RobotConstants.MASS_WITH_BUMPER_LBS))
			.withSimPeriod(Seconds.of(SimConstants.SIM_LOOP_PERIOD))
			.withRobotPlusBumperLength(Inches.of(RobotConstants.LENGTH_IN))
			.withWheelCOF(RobotConstants.WHEEL_COF)
			.withModuleLocations(null)
			.withPigeon(null)
			.withModules(null);
	}

	/**
	 * Apply the mass of the robot <strong>with bumpers.</strong>
	 * @param robotMassWithBumper robot mass with bumpers
	 * @return this config
	 */
	public SimSwerveDrivetrainConfig withRobotMass(Mass robotMassWithBumper) {
		robotMass = robotMassWithBumper;
		return this;
	}

	/**
	 * Applies the time it takes for a simulation periodic on the sim thread.
	 * @param simPeriodTime the period of time
	 * @return this config
	 */
	public SimSwerveDrivetrainConfig withSimPeriod(Time simPeriodTime) {
		simPeriod = simPeriodTime;
		return this;
	}

	/**
	 * Apply the length of the robot with the bumper on.
	 * @param robotPlusBumperLength length of the robot with bumper
	 * @return this config
	 */
	public SimSwerveDrivetrainConfig withRobotPlusBumperLength(Distance robotPlusBumperLength) {
		bumperLength = robotPlusBumperLength;
		return this;
	}

	/**
	 * Apply the length of the robot with the bumper on.
	 * @param robotPlusBumperWidth width of the robot with bumper
	 * @return this config
	 */
	public SimSwerveDrivetrainConfig withRobotPlusBumperWidth(Distance robotPlusBumperWidth) {
		bumperLength = robotPlusBumperWidth;
		return this;
	}

	/**
	 * Applies the drive motor used for each of the modules in the drivetrain.
	 * @param modulesDriveMotor the drive motor used in the module
	 * @return this config
	 */
	public SimSwerveDrivetrainConfig withModuleDriveMotor(DCMotor modulesDriveMotor) {
		moduleDriveMotor = modulesDriveMotor;
		return this;
	}

	/**
	 * Applies the steer motor used for each of the modules in the drivetrain.
	 * @param modulesSteerMotor the steer motor used in the module
	 * @return this config
	 */
	public SimSwerveDrivetrainConfig withModuleSteerMotor(DCMotor modulesSteerMotor) {
		moduleSteerMotor = modulesSteerMotor;
		return this;
	}

	/**
	 * Applies the Coefficient of Friction of the wheel material used on the modules.
	 * @param wheelCoefficientOfFriction Coefficient of Friction of the wheel
	 * @return this config
	 */
	public SimSwerveDrivetrainConfig withWheelCOF(double wheelCoefficientOfFriction) {
		wheelCOF = wheelCoefficientOfFriction;
		return this;
	}

	/**
	 * Applies the module locations of the drivetrain.
	 * @param moduleLocs the locations of the modules
	 * @return this config
	 */
	public SimSwerveDrivetrainConfig withModuleLocations(Translation2d[] moduleLocs) {
		moduleLocations = moduleLocs;
		return this;
	}

	/**
	 * Applies the pigeon IMU that is being used in this drivetrain.
	 * @param pigeonIMU the pigeon IMU
	 * @return this config
	 */
	public SimSwerveDrivetrainConfig withPigeon(Pigeon2 pigeonIMU) {
		pigeon = pigeonIMU;
		return this;
	}

	/**
	 * Applies all the modules to this drivetrain config.
	 * @param swerveModules the modules used on this drivetrain
	 * @return this config
	 */
	public SimSwerveDrivetrainConfig withModules(
		SwerveModule<TalonFX, TalonFX, CANcoder>[] swerveModules) {
		modules = swerveModules;
		return this;
	}

	/**
	 * Get the robot mass with bumpers.
	 * @return the robot mass with bumpers
	 */
	public Mass getRobotMass() {
		return robotMass;
	}

	/**
	 * Get the sim period time.
	 * @return the sim period time.
	 */
	public Time getSimPeriod() {
		return simPeriod;
	}

	/**
	 * Get the width of the robot with bumper.
	 * @return the with of the robot with the bumper on.
	 */
	public Distance getBumperWidth() {
		return bumperWidth;
	}

	/**
	 * Get the length of the robot with the bumper.
	 * @return the length of the robot with the bumper on.
	 */
	public Distance getBumperLength() {
		return bumperLength;
	}

	/**
	 * Get the drive motor used in the module.
	 * @return the drive motor type of the module
	 */
	public DCMotor getModuleDriveMotor() {
		return moduleDriveMotor;
	}

	/**
	 * Get the steer motor used in the module.
	 * @return the steer motor of the module
	 */
	public DCMotor getModuleSteerMotor() {
		return moduleSteerMotor;
	}

	/**
	 * Get the Coefficient of Friction of the module wheel.
	 * @return the COF of the wheel
	 */
	public double getWheelCOF() {
		return wheelCOF;
	}

	/**
	 * Get the locations of the modules.
	 * @return the locations of the modules.
	 */
	public Translation2d[] getModuleLocations() {
		return moduleLocations;
	}

	/**
	 * Get the pigeon IMU used in the drivetrain.
	 * @return the pigeon imu used in the module
	 */
	public Pigeon2 getPigeon() {
		return pigeon;
	}

	/**
	 * Get the swerve modules in the drivetrain.
	 * @return list of all the modules in the drivetrain
	 */
	public SwerveModule<TalonFX, TalonFX, CANcoder>[] getModules() {
		return modules;
	}
}
