package frc.robot.utils.simulation;

// CTRE Imports
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Pounds;
import edu.wpi.first.math.Pair;
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
// Maple-Sim
import org.ironmaple.simulation.drivesims.configs.BoundingCheck;
import frc.robot.constants.RobotConstants;
// Local Imports
import frc.robot.constants.SimConstants;




public class SimSwerveDrivetrainConfig {

	private Mass robotMass;
	private Time simPeriod;
	private Pair<Distance, Distance> bumperLengths;
	private Pair<DCMotor, DCMotor> moduleMotors;
	private double wheelCOF;
	private static Translation2d[] moduleLocations;
	private static Pigeon2 pigeon;
	private static SwerveModule<TalonFX, TalonFX, CANcoder>[] modules;

	public static SimSwerveDrivetrainConfig getDefault() {


		return new SimSwerveDrivetrainConfig()

			.withRobotMass(Pounds.of(RobotConstants.MASS_WITH_BUMPER_LBS))
			.withSimPeriod(Seconds.of(SimConstants.SIM_LOOP_PERIOD))
			.withBumperLengths(new Pair<Distance, Distance>(
				Inches.of(RobotConstants.LENGTH_IN),
				Inches.of(RobotConstants.WIDTH_IN)
			))
			.withModuleMotors(new Pair<DCMotor, DCMotor>(
				DCMotor.getKrakenX60(1),
				DCMotor.getKrakenX60(1)
			))

			.withWheelCOF(RobotConstants.WHEEL_COF)

			.withModuleLocations(moduleLocations)
			.withPigeon(pigeon)
			.withModules(modules);

	}

	public SimSwerveDrivetrainConfig withRobotMass(Mass robotMass) {
		this.robotMass = robotMass;
		checkRobotMass();
		return this;
	}

	public SimSwerveDrivetrainConfig withSimPeriod(Time simPeriod) {
		this.simPeriod = simPeriod;
		return this;
	}

	public SimSwerveDrivetrainConfig withBumperLengths(Pair<Distance, Distance> bumperLengths) {
		this.bumperLengths = bumperLengths;
		return this;
	}

	public SimSwerveDrivetrainConfig withModuleMotors(Pair<DCMotor, DCMotor> moduleMotors) {
		this.moduleMotors = moduleMotors;
		return this;
	}

	public SimSwerveDrivetrainConfig withWheelCOF(double wheelCOF) {
		this.wheelCOF = wheelCOF;
		return this;
	}

	public SimSwerveDrivetrainConfig withModuleLocations(Translation2d[] moduleLocations) {
		this.moduleLocations = moduleLocations;
		return this;
	}

	public SimSwerveDrivetrainConfig withPigeon (Pigeon2 pigeon) {
		this.pigeon = pigeon;
		return this;
	}

	public SimSwerveDrivetrainConfig withModules (SwerveModule<TalonFX, TalonFX, CANcoder>[] modules) {
		this.modules = modules;
		return this;
	}

	private void checkRobotMass() {
		BoundingCheck.check(robotMass.in(Kilograms), 110, 80, "robot mass", "kg");
	}


}
