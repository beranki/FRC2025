package frc.robot.utils;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.constants.Constants;

public final class CTREConfigs {
    public TalonFXConfiguration swerveTurnFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs() {
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.CANCODER_INVERTED;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveTurnFXConfig.MotorOutput.Inverted = Constants.Swerve.TURN_MOTOR_INVERTED;
        swerveTurnFXConfig.MotorOutput.NeutralMode = Constants.Swerve.TURNING_MOTOR_IDLE_MODE;

        /* Gear Ratio and Wrapping Config */
        swerveTurnFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.TURN_GEAR_RATIO;
        swerveTurnFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        /* Current Limiting */
        swerveTurnFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.TURN_ENABLE_CURRENT_LIMIT;
        swerveTurnFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.TURN_CURRENT_LIMIT;

        /* PID Config */
        swerveTurnFXConfig.Slot0.kP = Constants.Swerve.TURNING_P;
        swerveTurnFXConfig.Slot0.kI = Constants.Swerve.TURNING_I;
        swerveTurnFXConfig.Slot0.kD = Constants.Swerve.TURNING_D;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.Swerve.DRIVE_MOTOR_INVERTED;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.DRIVING_MOTOR_IDLE_MODE;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.DRIVE_GEAR_RATTIO;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.DRIVE_ENABLE_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.DRIVE_CURRENT_LIMIT;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.DRIVING_P;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.DRIVING_I;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.DRIVING_D;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.OPEN_LOOP_RAMP;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.CLOSED_LOOP_RAMP;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.CLOSED_LOOP_RAMP;
    }
}