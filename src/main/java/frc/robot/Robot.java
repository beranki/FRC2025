// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.AutoRoutines;
import frc.robot.constants.AutoConstants.AutoCommands;

// Systems
import frc.robot.systems.DriveFSMSystem;
// import frc.robot.systems.Mech1FSMSystem;
// import frc.robot.systems.Mech2FSMSystem;
// import frc.robot.systems.AutoHandlerSystem;
// import frc.robot.systems.AutoHandlerSystem.AutoPath;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;
	// Systems
	private DriveFSMSystem driveSystem;
	private AutoRoutines autoRoutines;
	private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
	private Command autoWorkflow;

	private static final Object[] PATH_1 = new Object[] {
		"S1_R2",
	};

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();


		// Instantiate all systems here
		if (HardwareMap.isDriveHardwarePresent()) {
			driveSystem = new DriveFSMSystem();
		}

		autoRoutines = new AutoRoutines(driveSystem);

		autoChooser.addOption("Path 1",
			autoRoutines.generateSequentialAutoWorkflow(PATH_1).cmd());
		SmartDashboard.putData("AUTO CHOOSER", autoChooser);
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		// autoHandler.reset(AutoPath.PATH1);
		autoWorkflow = getSelectedAutoRoutine();

		if (autoWorkflow != null) {
			autoWorkflow.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
		// autoHandler.update();
		CommandScheduler.getInstance().run();
		driveSystem.updateAutonomous();
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		driveSystem.reset();
	}

	@Override
	public void teleopPeriodic() {
		driveSystem.update(input);
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
	}

	@Override
	public void disabledPeriodic() {

	}

	@Override
	public void testInit() {
		System.out.println("-------- Test Init --------");
	}

	@Override
	public void testPeriodic() {

	}

	/* Simulation mode handlers, only used for simulation testing  */
	@Override
	public void simulationInit() {
		System.out.println("-------- Simulation Init --------");
	}

	@Override
	public void simulationPeriodic() { }

	// Do not use robotPeriodic. Use mode specific periodic methods instead.
	@Override
	public void robotPeriodic() { }

	/**
	 * Gets the autonomous command selected by the auto chooser.
	 *
	 * @return the selected autonomous command
	 */
	public Command getSelectedAutoRoutine() {
		return autoChooser.getSelected();
	}
}
