package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO _climberIO;
  private final ClimberIOInputsAutoLogged _climberInputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO climberIO) {
    this._climberIO = climberIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    _climberIO.updateInputs(_climberInputs);
    Logger.processInputs("Climber", _climberInputs);
  }

  /**
   * Set the climber motor speed
   *
   * @param speed Duty cycle percentage from -1.0 (full reverse) to 1.0 (full forward)
   */
  public void setClimberSpeed(double speed) {
    _climberIO.setMotorSpeed(speed);
  }

  /** Stop the climber motor */
  public void stopClimber() {
    _climberIO.setMotorSpeed(0.0);
  }
}
