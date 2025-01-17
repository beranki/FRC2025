package frc.robot.utils.simulation;

import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class MotorSimUtil {
	public static class TalonFXMotorControllerSim implements SimulatedMotorController {
		private final int id;
		private final TalonFXSimState talonFXSimState;

		/**
		 * Constructs a TalonFX MotorController which uses it's sim state.
		 * @param talonFX
		 */
		public TalonFXMotorControllerSim(TalonFX talonFX) {
			this.id = talonFX.getDeviceID();
			this.talonFXSimState = talonFX.getSimState();
		}

		@Override
		public Voltage updateControlSignal(
				Angle mechanismAngle,
				AngularVelocity mechanismVelocity,
				Angle encoderAngle,
				AngularVelocity encoderVelocity) {
			talonFXSimState.setRawRotorPosition(encoderAngle);
			talonFXSimState.setRotorVelocity(encoderVelocity);
			var voltage = SimulatedBattery.getBatteryVoltage();
			DogLog.log("Battery Voltage", voltage.toShortString());
			DogLog.log("Motor Voltage",
				talonFXSimState.getMotorVoltageMeasure().baseUnitMagnitude());
			talonFXSimState.setSupplyVoltage(voltage);

			return talonFXSimState.getMotorVoltageMeasure();
		}

		/**
		 * Gets the Id of this talonfx motor controller.
		 * @return the id of the talonfx motor controller
		 */
		public int getId() {
			return id;
		}
	}

	public static class TalonFXMotorControllerWithRemoteCanCoderSim extends
		TalonFXMotorControllerSim {
		private final int encoderId;
		private final CANcoderSimState remoteCancoderSimState;

		/**
		 * Constructs a TalonFX motor controller but with a cancoder sim.
		 * @param talonFX motor to use
		 * @param cancoder cancoder used for sim state
		 */
		public TalonFXMotorControllerWithRemoteCanCoderSim(
			TalonFX talonFX,
			CANcoder cancoder
		) {
			super(talonFX);
			this.remoteCancoderSimState = cancoder.getSimState();

			this.encoderId = cancoder.getDeviceID();
		}

		@Override
		public Voltage updateControlSignal(
				Angle mechanismAngle,
				AngularVelocity mechanismVelocity,
				Angle encoderAngle,
				AngularVelocity encoderVelocity) {
			remoteCancoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
			remoteCancoderSimState.setRawPosition(mechanismAngle);
			remoteCancoderSimState.setVelocity(mechanismVelocity);

			return super.updateControlSignal(
				mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
		}
		/**
		 * Get the identifier of the encoder paired with this motor.
		 * @return the identifier of this sim motor
		 */
		public int getEncoderID() {
			return encoderId;
		}
	}

}
