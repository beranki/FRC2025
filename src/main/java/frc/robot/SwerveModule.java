package frc.robot;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utils.CTREConfigs;
import frc.robot.utils.Conversions;
import frc.robot.constants.Constants;

public class SwerveModule {
    private Rotation2d angleOffset;

    private TalonFX turnMotor;
    private TalonFX driveMotor;
    private CANcoder turningEncoder;

    private final SimpleMotorFeedforward driveFeedForward =
            new SimpleMotorFeedforward(
                Constants.Swerve.DRIVING_S,
                Constants.Swerve.DRIVING_V, 
                Constants.Swerve.DRIVING_A);

    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    private final PositionVoltage anglePosition = new PositionVoltage(0);

    private final CTREConfigs ctreConfigs = new CTREConfigs();

    public SwerveModule(int driveID, int turnID, int canID, double chassisAngularOffset) {
        angleOffset = new Rotation2d(chassisAngularOffset);

        turningEncoder = new CANcoder(canID);
        turningEncoder.getConfigurator().apply(ctreConfigs.swerveCANcoderConfig);

        turnMotor = new TalonFX(turnID);
        turnMotor.getConfigurator().apply(ctreConfigs.swerveTurnFXConfig);
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