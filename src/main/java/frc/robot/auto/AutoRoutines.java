package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.constants.AutoConstants;
import frc.robot.systems.DriveFSMSystem;

public class AutoRoutines {
	// Initialize all auto factories here
	private AutoFactory driveAutoFactory;
	private AutoFactory elevatorAutoFactory;
	private AutoFactory intakeAutoFactory;

	// Initialize all FSMs (with commands) here
	private DriveFSMSystem driveSystem;

	/**
	 * Constructs an AutoRoutines object with the specified AutoFactory.
	 * @param system1
	 * */
	public AutoRoutines(DriveFSMSystem system1) {
		driveSystem = system1;
		driveAutoFactory = driveSystem.createAutoFactory();
	}

	/**
	 * Creates and returns a auto routine.
	 * S1 -> R2 (Tag + Score) -> Source (Tag + Intake) -> R5 (Tag + Score)
	 * @return the auto routine
	 */
	public AutoRoutine bS1R2StationR5() {
		final AutoRoutine routine = driveAutoFactory.newRoutine("testPath");
		final AutoTrajectory path1 = routine.trajectory("S1_R2");
		final AutoTrajectory path2 = routine.trajectory("R2_Station");
		final AutoTrajectory path3 = routine.trajectory("Station_R5");

		routine.active().onTrue(
			path1.resetOdometry()
			.andThen(path1.cmd())
			.andThen(driveSystem.alignToReefTagCommand(
				AutoConstants.REEF_2_TAG_ID,
				AutoConstants.REEF_2_X_L_TAG_OFFSET,
				AutoConstants.REEF_2_Y_L_TAG_OFFSET))
			.andThen(path2.cmd())
			// .andThen(driveSystem.alignToSourceTagCommand(
			// 	AutoConstants.BLUE_STATION_ID,
			// 	AutoConstants.SOURCE_X_OFFSET,
			// 	AutoConstants.SOURCE_Y_OFFSET
			// ))
			.andThen(path3.cmd())
			.andThen(driveSystem.brakeCommand())
		);
		return routine;
	}
}
