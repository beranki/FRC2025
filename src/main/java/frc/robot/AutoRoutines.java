package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
	private AutoFactory autoFactory;

	/**
	 * Constructs an AutoRoutines object with the specified AutoFactory.
	 *
	 * @param factory the AutoFactory to use for creating auto routines
	 */
	public AutoRoutines(AutoFactory factory) {
		autoFactory = factory;
	}

	/**
	 * Creates and returns a test auto routine.
	 *
	 * @return the test auto routine
	 */
	public AutoRoutine testAuto() {
		final AutoRoutine routine = autoFactory.newRoutine("testPath");
		final AutoTrajectory path = routine.trajectory("testPath");

		routine.active().onTrue(
			path.resetOdometry()
			.andThen(path.cmd())
		);
		return routine;
	}
}
