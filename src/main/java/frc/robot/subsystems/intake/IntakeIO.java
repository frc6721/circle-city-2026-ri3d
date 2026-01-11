package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    // |================= START INTAKE PIVOT MOTOR LOGGING =================|
    public Temperature _intakePivotMotorTemperature;
    public AngularVelocity _intakePivotMotorVelocity;
    public Angle _intakePivotMotorPosition;
    public Voltage _intakePivotMotorVoltage;
    public Current _intakePivotMotorCurrent;
    // |================= END INTAKE PIVOT MOTOR LOGGING =================|

    // |================= START INTAKE ROLLER MOTOR LOGGING =================|
    public Temperature _intakeRollerMotorTemperature;
    public AngularVelocity _intakeRollerMotorVelocity;
    public Angle _intakeRollerMotorPosition;
    public Voltage _intakeRollerMotorVoltage;
    public Current _intakeRollerMotorCurrent;
    // |================= END INTAKE ROLLER MOTOR LOGGING =================|

    // |================= START LIMIT SWITCH LOGGING =================|
    public boolean _upperLimitSwitchTriggered;
    // |================= END LIMIT SWITCH LOGGING =================|

  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  // |============================== PIVOT MOTOR METHODS ============================== |
  public default void setPivotTargetPosition(Angle angle) {}

  public default void setPivotMotorSpeed(AngularVelocity speed) {}

  public default void zeroPivotEncoder() {}

  // |============================== ROLLER MOTOR METHODS ============================== |
  public default void setRollerMotorOutput(double output) {}
}
