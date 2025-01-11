package frc.robot.auto;

import java.io.File;
import java.util.HashMap;
import java.util.List;
import java.util.Stack;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutoConstants;
import frc.robot.systems.DriveFSMSystem;

public class AutoRoutines {
	// Initialize all auto factories here
	private AutoFactory driveAutoFactory;
	private AutoFactory elevatorAutoFactory;
	private AutoFactory intakeAutoFactory;

	// Initialize all FSMs (with commands) here
	private DriveFSMSystem driveSystem;


	// Initialize all paths
	private final AutoRoutine driveRoutine = driveAutoFactory.newRoutine("DriveRoutine");

	private HashMap<String, AutoTrajectory> paths;
	private Stack<String> autoStageStack;

	private final AutoTrajectory pathS1R2 = driveRoutine.trajectory("S1_R2");
	private final AutoTrajectory pathR2Station = driveRoutine.trajectory("R2_Station");
	private final AutoTrajectory pathStationR5 = driveRoutine.trajectory("Station_R5");

	/**
	 * Constructs an AutoRoutines object with the specified AutoFactory.
	 * @param system1
	 * */
	public AutoRoutines(DriveFSMSystem system1) {
		driveSystem = system1;
		driveAutoFactory = driveSystem.createAutoFactory();
		paths = new HashMap<String, AutoTrajectory>();
	}

	/**
	 * Dynamically generate all paths for drive routine in a map.
	 */
	public void generateDriveRoutineMap(File deployDir) {
		for (File choreoFile : deployDir.listFiles()) {
			if (choreoFile.getName().endsWith(".traj")) {
				paths.put(choreoFile.getName(), driveRoutine.trajectory(choreoFile.getName()));
			}
		}
	}

	/**
	 * Creates and returns a auto routine.
	 * S1 -> R2 (Tag + Score) -> Source (Tag + Intake) -> R5 (Tag + Score)
	 * @return the auto routine
	 */
	public AutoRoutine generateSequentialAutoWorkflow(List<?> autoStageSupply) {

		Command seqInstruction;


		for (var autoStage : autoStageSupply) {
			/* Processing drive traj */
			if (autoStage.getClass().equals(String.class) && paths.containsKey(autoStage)) {
				
			}
			/* Processing mech command traj */
		}


		Command seqInstruction;

		driveRoutine.active().onTrue(
			pathS1R2.resetOdometry()
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
