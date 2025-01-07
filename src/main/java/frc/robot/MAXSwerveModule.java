package frc.robot;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utils.CTREConfigs;
import frc.robot.utils.Conversions;
import frc.robot.utils.SparkMaxConfigs;
import frc.robot.constants.Constants;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.SwerveConstants.ModuleConstants;

public class MAXSwerveModule {
    private Rotation2d angleOffset;

    private SparkMax turnMotor;
    private SparkMax driveMotor;

    private RelativeEncoder drivingEncoder;
    private AbsoluteEncoder turningEncoder;

    private final SimpleMotorFeedforward driveFeedForward =
            new SimpleMotorFeedforward(
                ModuleConstants.DRIVING_S,
                ModuleConstants.DRIVING_V, 
                ModuleConstants.DRIVING_A);

    public MAXSwerveModule(int driveID, int turnID, double chassisAngularOffset) {
        angleOffset = new Rotation2d(chassisAngularOffset);

        turnMotor = new SparkMax(turnID, MotorType.kBrushless);
        driveMotor = new SparkMax(driveID, MotorType.kBrushless);

        drivingEncoder = driveMotor.getEncoder();
        turningEncoder = turnMotor.getAbsoluteEncoder();

        AbsoluteEncoderConfig con = new AbsoluteEncoderConfig();
        SparkBaseConfig c2 = new SparkBaseConfig();
        c2.

        resetToAbsolute();

        driveMotor = new TalonFX(driveID);
        driveMotor.getConfigurator().apply(ctreConfigs.swerveDriveFXConfig);
        driveMotor.getConfigurator().setPosition(0.0);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState.optimize(getState().angle);
        turnMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED_METERS;
            driveMotor.setControl(driveDutyCycle);
        } else {
            driveVelocity.Velocity =
                    Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.WHEEL_CIRCUMFERENCE);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            driveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder() {
        return new Rotation2d(turningEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute() {
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        turnMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.RPSToMPS(
                        driveMotor.getVelocity().getValue().in(RotationsPerSecond),
                        Constants.Swerve.WHEEL_CIRCUMFERENCE),
                new Rotation2d(turnMotor.getPosition().getValue()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.rotationsToMeters(
                        driveMotor.getPosition().getValue().in(Rotations), Constants.Swerve.WHEEL_CIRCUMFERENCE),
                new Rotation2d(turnMotor.getPosition().getValue()));
    }
}