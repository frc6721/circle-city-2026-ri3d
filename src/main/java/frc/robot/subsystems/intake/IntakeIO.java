package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    // |================= START RIGHT INTAKE PIVOT MOTOR LOGGING =================|
    public Temperature _intakeRightPivotMotorTemperature;
    public AngularVelocity _intakeRightPivotMotorVelocity;
    public Rotation2d _intakeRightPivotMotorPosition;
    public Voltage _intakeRightPivotMotorVoltage;
    public Current _intakeRightPivotMotorCurrent;
    // |================= END RIGHT INTAKE PIVOT MOTOR LOGGING =================|

    // |================= START LEFT INTAKE PIVOT MOTOR LOGGING =================|
    public Temperature _intakeLeftPivotMotorTemperature;
    public AngularVelocity _intakeLeftPivotMotorVelocity;
    public Rotation2d _intakeLeftPivotMotorPosition;
    public Voltage _intakeLeftPivotMotorVoltage;
    public Current _intakeLeftPivotMotorCurrent;
    // |================= END LEFT INTAKE PIVOT MOTOR LOGGING =================|

    // |================= START INTAKE ROLLER MOTOR LOGGING =================|
    public Temperature _intakeRollerMotorTemperature;
    public AngularVelocity _intakeRollerMotorVelocity;
    public Voltage _intakeRollerMotorVoltage;
    public Current _intakeRollerMotorCurrent;
    // |================= END INTAKE ROLLER MOTOR LOGGING =================|
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  // |============================== PIVOT MOTOR METHODS ============================== |
  public void setPivotTargetPosition(Rotation2d angle);

  public default void setIntakePivotDutyCucleOutput(double output) {}

  // |============================== ROLLER MOTOR METHODS ============================== |
  public default void setRollerMotorOutput(double output) {}
}
