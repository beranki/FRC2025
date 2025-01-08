package frc.robot.utils;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.SwerveConstants.ModuleConstants;

public class SparkConfigs {

        public SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public SparkMaxConfig turningConfig = new SparkMaxConfig();
        

        public SparkConfigs() {
                // Use module constants to calculate conversion factors and feed forward gain.

                drivingConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(ModuleConstants.DRIVING_MOTOR_CUTTENT_LIMIT);
                drivingConfig.encoder
                        .positionConversionFactor(ModuleConstants.DRIVING_ENCODER_POSITION_FACTOR) // meters
                        .velocityConversionFactor(ModuleConstants.DRIVING_ENCODOR_VELOCITY_FACTOR); // meters per second
                drivingConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(ModuleConstants.DRIVING_P, ModuleConstants.DRIVING_I, ModuleConstants.DRIVING_D)
                        .velocityFF(ModuleConstants.DRIVING_FF)
                        .outputRange(ModuleConstants.DRIVING_MIN_OUTPUT, ModuleConstants.DRIVING_MAX_OUTPUT);

                turningConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(ModuleConstants.TURNING_MOTOR_CURRENT_LIMIT);
                turningConfig.absoluteEncoder
                        // Invert the turning encoder, since the output shaft rotates in the opposite
                        // direction of the steering motor in the MAXSwerve Module.
                        .inverted(true)
                        .positionConversionFactor(ModuleConstants.TURNING_ENCODER_POSITION_FACTOR) // radians
                        .velocityConversionFactor(ModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR); // radians per second
                turningConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(ModuleConstants.TURNING_P, ModuleConstants.TURNING_I, ModuleConstants.TURNING_D)
                        .outputRange(ModuleConstants.TURNING_MIN_OUTPUT, ModuleConstants.TURNING_MAX_OUTPUT)
                        // Enable PID wrap around for the turning motor. This will allow the PID
                        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                        // to 10 degrees will go through 0 rather than the other direction which is a
                        // longer route.
                        .positionWrappingEnabled(true)
                        .positionWrappingInputRange(0, ModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
        }
}