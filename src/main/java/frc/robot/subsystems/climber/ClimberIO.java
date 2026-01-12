package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    // |================= START CLIMBER MOTOR LOGGING =================|
    public Temperature _climberMotorTemperature;
    public Voltage _climberMotorVoltage;
    public Current _climberMotorCurrent;
    // |================= END CLIMBER MOTOR LOGGING =================|
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  // |============================== MOTOR METHODS ============================== |
  /** Set the climber motor speed as a duty cycle percentage (-1.0 to 1.0) */
  public default void setMotorSpeed(double speed) {}
}
