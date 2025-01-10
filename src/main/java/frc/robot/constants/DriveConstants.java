package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;

public class DriveConstants {
	public static final double DRIVE_DEADBAND = 0.1;
	public static final double ROTATION_DEADBAND = 0.1;

	public static final double MAC_ANGULAR_VELO_RPS = 0.75;
	public static final PIDConstants AUTO_TRANSLATION_PID = new PIDConstants(10, 0, 0);
	public static final PIDConstants AUTO_ROTATION_PID = new PIDConstants(7, 0, 0);
}
