package frc.robot.subsystems.feeder.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.feeder.FeederConstants;

/**
 * Simulation implementation of FeederIO.
 *
 * <p>This is a simple simulation without physics - it just provides voltage and current logging for
 * the feeder motor. The feeder doesn't need realistic physics since it's just duty cycle
 * controlled.
 *
 * <p>This class simulates:
 *
 * <ul>
 *   <li><b>Motor:</b> Uses REV SparkMaxSim for realistic motor behavior
 *   <li><b>Current draw:</b> Approximated based on duty cycle
 *   <li><b>Battery effects:</b> Updates RoboRIO voltage based on current draw
 * </ul>
 */
public class SimFeederIO implements FeederIO {

  // Simulated motor
  private final SparkMax feederMotor;

  // REV simulation wrapper
  private final SparkMaxSim feederSim;

  // Control state
  private double dutyCycle = 0.0;

  /**
   * Creates a new SimFeederIO.
   *
   * <p>Initializes simulated motor using constants from FeederConstants.
   */
  public SimFeederIO() {
    // Create simulated motor (using arbitrary CAN ID since it doesn't matter in sim)
    feederMotor = new SparkMax(70, MotorType.kBrushless);

    // Configure the feeder motor
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(FeederConstants.FEEDER_MOTOR_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(FeederConstants.FEEDER_MOTOR_SMART_CURRENT_LIMIT)
        .voltageCompensation(12.0);

    feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Create REV simulation wrapper
    feederSim = new SparkMaxSim(feederMotor, FeederConstants.FEEDER_MOTOR);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    // Calculate simulated values based on duty cycle
    double appliedVoltage = dutyCycle * RoboRioSim.getVInVoltage();

    // Approximate current draw based on duty cycle (no load = low current, full load = high
    // current)
    double simulatedCurrent = Math.abs(dutyCycle) * 15.0; // ~15A at full power

    // Update battery simulation
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(simulatedCurrent));

    // Update the SparkMax simulation
    feederSim.setBusVoltage(RoboRioSim.getVInVoltage());
    feederSim.iterate(
        dutyCycle * 5000.0, // Approximate RPM at full speed
        RoboRioSim.getVInVoltage(),
        0.02);

    // Populate inputs
    inputs._feederMotorTemperature = Celsius.of(35.0); // Simulated constant temp
    inputs._feederMotorVoltage = Volts.of(appliedVoltage);
    inputs._feederMotorCurrent = Amps.of(simulatedCurrent);
  }

  @Override
  public void setMotorSpeed(double speed) {
    dutyCycle = Math.max(-1.0, Math.min(1.0, speed));
    feederMotor.set(dutyCycle);
  }
}
