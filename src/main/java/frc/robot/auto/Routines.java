package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class Routines {
	private final AutoFactory factory;

	public Routines(AutoFactory mfactory) {
		factory = mfactory;
	}

	public AutoRoutine auto1() {
		final AutoRoutine routine = factory.newRoutine("routine");
		final AutoTrajectory R2__Station = routine.trajectory("R2_StationL");
		final AutoTrajectory S1_R2 = routine.trajectory("S1_R2");
		final AutoTrajectory Station_R3 = routine.trajectory("StationL_R3");

		routine.active().onTrue(
			S1_R2.resetOdometry()
			.andThen(R2__Station.cmd())
			.andThen(Station_R3.cmd())
		);

		return routine;
	}
}
