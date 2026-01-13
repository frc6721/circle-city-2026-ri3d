package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {

    // |================= START FLYWHEEL MOTOR LOGGING =================|
    public Temperature _flywheelMotorTemperature;
    public AngularVelocity _flywheelMotorVelocity;
    public Voltage _flywheelMotorVoltage;
    public Current _flywheelMotorCurrent;
    // |================= END FLYWHEEL MOTOR LOGGING =================|

  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  // |============================== FLYWHEEL MOTOR METHODS ============================== |

  public default void setFlywheelSpeed(AngularVelocity speed) {}

  public default void setFlyWheelDutyCycle(double output) {}

  public default void stopFlywheel() {}
}
