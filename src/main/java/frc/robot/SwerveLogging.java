package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class SwerveLogging {

	private double maxSpeed;

	/**
	 * Construct a telemetry object, with the specified max speed of the robot.
	 * @param maxSPeed Maximum speed in meters per second
	 */
	public SwerveLogging(double maxSPeed) {
		this.maxSpeed = maxSPeed;
		SignalLogger.start();
	}

	/* What to publish over networktables for telemetry */
	private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

	/* Robot swerve drive state */
	private final NetworkTable driveStateTable = inst.getTable("DriveState");
	private final StructPublisher<Pose2d>
		drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
	private final StructPublisher<ChassisSpeeds>
		driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
	private final StructArrayPublisher<SwerveModuleState>
		driveModuleStates = driveStateTable
				.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
	private final StructArrayPublisher<SwerveModuleState>
		driveModuleTargets = driveStateTable
				.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
	private final StructArrayPublisher<SwerveModulePosition>
		driveModulePositions = driveStateTable
				.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
	private final DoublePublisher
		driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
	private final DoublePublisher
		driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

	/* Robot pose for field positioning */
	private final NetworkTable table = inst.getTable("Pose");
	private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
	private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

	/* Mechanisms to represent the swerve module states */
	private final Mechanism2d[] moduleMechanisms = new Mechanism2d[] {
		new Mechanism2d(1, 1),
		new Mechanism2d(1, 1),
		new Mechanism2d(1, 1),
		new Mechanism2d(1, 1),
	};

	private static final double MECHANISM_ROOT_COORD = 0.5;

	/* A direction and length changing ligament for speed representation */
	private final MechanismLigament2d[] moduleSpeeds = new MechanismLigament2d[] {
		moduleMechanisms[0].getRoot(
			"RootSpeed",
			MECHANISM_ROOT_COORD,
			MECHANISM_ROOT_COORD)
				.append(new MechanismLigament2d("Speed", 0.5, 0)),

		moduleMechanisms[1].getRoot(
			"RootSpeed",
			MECHANISM_ROOT_COORD,
			MECHANISM_ROOT_COORD).append(new MechanismLigament2d("Speed", 0.5, 0)),

		moduleMechanisms[2].getRoot(
			"RootSpeed",
			MECHANISM_ROOT_COORD,
			MECHANISM_ROOT_COORD).append(new MechanismLigament2d("Speed", 0.5, 0)),

		moduleMechanisms[2 + 1].getRoot(
			"RootSpeed",
			MECHANISM_ROOT_COORD,
			MECHANISM_ROOT_COORD).append(new MechanismLigament2d("Speed", 0.5, 0)),
	};
	/* A direction changing and length constant ligament for module direction */
	private final MechanismLigament2d[] moduleDirections = new MechanismLigament2d[] {
		moduleMechanisms[0].getRoot("RootDirection", MECHANISM_ROOT_COORD, MECHANISM_ROOT_COORD)
			.append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
		moduleMechanisms[1].getRoot("RootDirection", MECHANISM_ROOT_COORD, MECHANISM_ROOT_COORD)
			.append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
		moduleMechanisms[2].getRoot("RootDirection", MECHANISM_ROOT_COORD, MECHANISM_ROOT_COORD)
			.append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
		moduleMechanisms[2 + 1].getRoot("RootDirection", MECHANISM_ROOT_COORD, MECHANISM_ROOT_COORD)
			.append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
	};

	private final int poseArrayLen = 3;
	private final int moduleStatesArrayLen = 8;
	private final int moduleTargetArrayLen = 8;
	private final double[]
					poseArray = new double[poseArrayLen];
	private final double[]
					moduleStatesArray = new double[moduleStatesArrayLen];
	private final double[]
					moduleTargetArray = new double[moduleTargetArrayLen];

	/**
	 * Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger.
	 * @param state the current swerve drive state
	 */
	public void applyStateLogging(SwerveDriveState state) {
		/* Telemeterize the swerve drive state */
		drivePose.set(state.Pose);
		driveSpeeds.set(state.Speeds);
		driveModuleStates.set(state.ModuleStates);
		driveModuleTargets.set(state.ModuleTargets);
		driveModulePositions.set(state.ModulePositions);
		driveTimestamp.set(state.Timestamp);
		driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

		/* Also write to log file */
		poseArray[0] = state.Pose.getX();
		poseArray[1] = state.Pose.getY();
		poseArray[2] = state.Pose.getRotation().getDegrees();
		for (int i = 0; i < 2 + 2; ++i) {
			moduleStatesArray[i * 2 + 0] = state.ModuleStates[i].angle.getRadians();
			moduleStatesArray[i * 2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
			moduleTargetArray[i * 2 + 0] = state.ModuleTargets[i].angle.getRadians();
			moduleTargetArray[i * 2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
		}

		SignalLogger.writeDoubleArray("DriveState/Pose", poseArray);
		SignalLogger.writeDoubleArray("DriveState/ModuleStates", moduleStatesArray);
		SignalLogger.writeDoubleArray("DriveState/ModuleTargets", moduleTargetArray);
		SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

		/* Telemeterize the pose to a Field2d */
		fieldTypePub.set("Field2d");
		fieldPub.set(poseArray);

		/* Telemeterize the module states to a Mechanism2d */
		for (int i = 0; i < 2 + 2; ++i) {
			moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
			moduleDirections[i].setAngle(state.ModuleStates[i].angle);
			moduleSpeeds[i].setLength(
				state.ModuleStates[i].speedMetersPerSecond / (2 * maxSpeed));

			SmartDashboard.putData("Module " + i, moduleMechanisms[i]);
		}
	}
}
