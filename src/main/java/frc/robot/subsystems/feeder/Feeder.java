package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {

  private final FeederIO _feederIO;
  private final FeederIOInputsAutoLogged _feederInputs = new FeederIOInputsAutoLogged();

  /**
   * Creates a new Feeder subsystem.
   *
   * @param feederIO Hardware abstraction implementation (real or sim)
   */
  public Feeder(FeederIO feederIO) {
    this._feederIO = feederIO;
  }

  @Override
  public void periodic() {
    // Update and log sensor inputs from hardware
    _feederIO.updateInputs(_feederInputs);
    Logger.processInputs("Feeder", _feederInputs);
  }

  /**
   * Sets the feeder motor speed.
   *
   * @param speed Duty cycle from -1.0 (full reverse) to 1.0 (full forward)
   */
  public void setFeederSpeed(double speed) {
    _feederIO.setMotorSpeed(speed);
  }

  /** Stops the feeder motor. */
  public void stop() {
    _feederIO.setMotorSpeed(0.0);
  }
}
