package frc.robot.logging;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.Constants;

public final class MechLogging {
	private Pose3d primaryElevatorPose;
	private Pose3d secondaryElevatorPose;
	private Pose3d climberPose;
	private Pose3d drivePose;

	private static MechLogging instance = new MechLogging();

	private MechLogging() {
		primaryElevatorPose = new Pose3d();
		secondaryElevatorPose = new Pose3d();
		climberPose = new Pose3d();
		drivePose = new Pose3d();
	}

	/**
	 * Get the instance of the singleton class.
	 * @return the instance
	 */
	public static MechLogging getInstance() {
		return instance;
	}

	/**
	 * Generates the pose for the elevator based on encoder position.
	 * @param encoderSimPosition the simulated location of the elevator motor encoder.
	 */
	public void updateElevatorPose3d(double encoderSimPosition) {
		double height = encoderSimPosition * Constants.INCHES_TO_METERS;

		var pose = new Pose3d(
			new Translation3d(0, 0, height),
			Rotation3d.kZero
		);

		primaryElevatorPose = pose;
		secondaryElevatorPose = pose.div(2);
	}

	/**
	 * Updates the pose for the climber based on encoder position.
	 * @param encoderSimPosition the simulated location of the climber motor encoder.
	 */
	public void updatesClimberPose3d(Angle encoderSimPosition) {
		climberPose = new Pose3d(
			Translation3d.kZero,
			new Rotation3d(0, encoderSimPosition.in(Radians), 0)
		);
	}

	/**
	 * Get the primary elevator pose.
	 * @return the pose of the inner part of the elevator
	 */
	public Pose3d getPrimaryElevatorPose() {
		return primaryElevatorPose.relativeTo(drivePose);
	}

	/**
	 * Get the secondary elevator pose.
	 * @return the pose of the inner-most part of the elevator
	 */
	public Pose3d getSecondaryElevatorPose() {
		return secondaryElevatorPose.relativeTo(drivePose);
	}

	/**
	 * Get the pose of the climber ligament.
	 * @return pose of the rotating climber ligament.
	 */
	public Pose3d getClimberPose() {
		return climberPose.relativeTo(drivePose);
	}

	/**
	 * Getter for the array of poses we want to simulate.
	 * @return the array of poses to display in advantageScope
	 */
	public Pose3d[] getRobotPoses() {
		return new Pose3d[]{
			drivePose,
			getPrimaryElevatorPose(),
			getSecondaryElevatorPose(),
			getClimberPose()
		};
	}

	/**
	 * Sets the drive pose data, used to determine the components' absolute location.
	 * @param pose the 2d pose of the robot
	 */
	public void setDrivePoseData(Pose2d pose) {
		this.drivePose = new Pose3d(pose);
	}
}
