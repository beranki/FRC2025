package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.utils.SparkConfigs;

public class SwerveModuleSim {
    private Rotation2d angleOffset;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    private SparkMax turnMotor;
    private SparkMax driveMotor;
    private SparkMaxSim driveMotorSim;
    private SparkMaxSim turnMotorSim;

    private SparkRelativeEncoderSim drivingEncoder;
    private SparkAbsoluteEncoderSim turningEncoder;
    
    private final SparkConfigs sparkConfigs = new SparkConfigs();

    private final SparkClosedLoopController driveClosedLoopController;
    private final SparkClosedLoopController turnClosedLoopController;

    public SwerveModuleSim(int driveID, int turnID, double chassisAngularOffset) {
        driveMotor = new SparkMax(driveID, MotorType.kBrushless);
        turnMotor = new SparkMax(turnID, MotorType.kBrushless);

        driveMotor.configure(sparkConfigs.drivingConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        turnMotor.configure(sparkConfigs.turningConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        driveClosedLoopController = driveMotor.getClosedLoopController();
        turnClosedLoopController = turnMotor.getClosedLoopController();

        driveMotorSim = new SparkMaxSim(driveMotor, DCMotor.getNEO(1));
        turnMotorSim = new SparkMaxSim(turnMotor, DCMotor.getNeo550(1));

        drivingEncoder = driveMotorSim.getRelativeEncoderSim();
        turningEncoder = turnMotorSim.getAbsoluteEncoderSim();

        angleOffset = Rotation2d.fromRadians(chassisAngularOffset);
        desiredState.angle = new Rotation2d(turningEncoder.getPosition());
        drivingEncoder.setPosition(0);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
      return new SwerveModuleState(drivingEncoder.getVelocity(),
          new Rotation2d(turningEncoder.getPosition()).minus(angleOffset));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
      return new SwerveModulePosition(
          drivingEncoder.getPosition(),
          new Rotation2d(turningEncoder.getPosition()).minus(angleOffset));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(angleOffset);

        correctedDesiredState.optimize(new Rotation2d(turningEncoder.getPosition()));

        driveClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        turnClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

        this.desiredState = desiredState;
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
      drivingEncoder.setPosition(0);
    }
}