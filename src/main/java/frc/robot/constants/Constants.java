package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.COTSTalonFXSwerveConstants;

public class Constants {
    public class Swerve {
        public static final COTSTalonFXSwerveConstants chosenModule =
                COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        public static final double TRACK_WIDTH = Units.inchesToMeters(22.5);
        public static final double WHEEL_BASE = Units.inchesToMeters(22.75);

        public static final double WHEEL_DIAMETER = chosenModule.wheelDiameter;
        public static final double WHEEL_CIRCUMFERENCE = chosenModule.wheelCircumference;

        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
        );

        public static final double FRONT_LEFT_ANGLE_OFFSET = - Math.PI / 2;
        public static final double FRONT_RIGHT_ANGLE_OFFSET = 0;
        public static final double BACK_LEFT_ANGLE_OFFSET = Math.PI;
        public static final double BACK_RIGHT_ANGLE_OFFSET = Math.PI / 2;

        public static final NeutralModeValue DRIVING_MOTOR_IDLE_MODE = NeutralModeValue.Brake;
		public static final NeutralModeValue TURNING_MOTOR_IDLE_MODE = NeutralModeValue.Coast;

		public static final double DRIVING_P = 0.04;
		public static final double DRIVING_I = 0;
		public static final double DRIVING_D = 0;
		public static final double DRIVING_FF = 0;

        public static final double DRIVING_S = 0;
        public static final double DRIVING_V = 0;
        public static final double DRIVING_A = 0;

		public static final double TURNING_P = chosenModule.angleKP;
		public static final double TURNING_I = chosenModule.angleKI;
		public static final double TURNING_D = chosenModule.angleKD;

        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        public static final int DRIVE_CURRENT_LIMIT = 50;
        public static final int DRIVE_CURRENT_THRESHOLD = 60;
        public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        public static final int TURN_CURRENT_LIMIT = 25;
        public static final int TURN_CURRENT_THRESHOLD = 40;
        public static final double TURN_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean TURN_ENABLE_CURRENT_LIMIT = true;

        public static final double DRIVE_GEAR_RATTIO = chosenModule.driveGearRatio;
        public static final double TURN_GEAR_RATIO = chosenModule.angleGearRatio;

        public static final InvertedValue TURN_MOTOR_INVERTED = chosenModule.angleMotorInvert;
        public static final InvertedValue DRIVE_MOTOR_INVERTED = chosenModule.driveMotorInvert;

        public static final SensorDirectionValue CANCODER_INVERTED = chosenModule.cancoderInvert;
    
        public static final double MAX_SPEED_METERS = 6;
        public static final double MAX_ANGULAR_VELOCITY = 2 * Math.PI;
    }

    public class OIConstants {
        public static final double DRIVE_DEADBAND = 0.2;
    }

    public class VisionConstants {
        public static final double CAMERA_HEIGHT_METERS = 0.5;
        public static final double TARGET_HEIGHT_METERS = 1.435;
        public static final double CAMERA_PITCH_DEGREES = -30;

        public static final double VISION_DES_ANGLE_DEGREES = 0;
        public static final double VISION_DES_RANGE_METERS = 0.5;
        public static final double VISION_TURN_P = 0.1;
        public static final double VISION_STRAFE_P = 0.1;
    }
}
