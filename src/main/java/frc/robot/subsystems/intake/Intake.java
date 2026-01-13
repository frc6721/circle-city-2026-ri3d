package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Intake extends SubsystemBase {

  private final IntakeIO _intakeIO;
  private final IntakeIOInputsAutoLogged _intakeInputs = new IntakeIOInputsAutoLogged();
  private IntakePosition _intakePosition;

  public enum IntakePosition {
    STOW(IntakeConstants.POSITION_STOW),
    PICKUP(IntakeConstants.POSITION_PICKUP);

    private final LoggedNetworkNumber _angle;

    /**
     * These enum represents a prefedined position of the intake. These are what we pass to commands
     * to set the intake positions
     *
     * @param angle
     */
    private IntakePosition(LoggedNetworkNumber angle) {
      this._angle = angle;
    }

    /**
     * Return the angle associated with a given position. These angles are defined in
     * IntakeConstants.java
     *
     * @return
     */
    public Angle getAngle() {
      return Degrees.of(this._angle.get());
    }
  }

  public Intake(IntakeIO intakeIO) {
    this._intakeIO = intakeIO;

    // assume that the intake is all the way up when first turned on
    _intakePosition = IntakePosition.STOW;
  }

  @Override
  public void periodic() {
    _intakeIO.updateInputs(_intakeInputs);
    Logger.processInputs("Intake", _intakeInputs);

    // LOGGING
    Logger.recordOutput(
        "Intake/Current-Pivot-Angle", _intakeInputs._intakeRightPivotMotorPosition.getDegrees());
    Logger.recordOutput("Intake/Desired-Pivot-Angle", _intakePosition.getAngle().in(Degrees));
  }

  public void setIntakePosition(IntakePosition position) {
    _intakePosition = position;
    _intakeIO.setPivotTargetPosition(position.getAngle());
  }


  public void setIntakePivotDutyCucleOutput(double output) {
    _intakeIO.setIntakePivotDutyCucleOutput(output);
  }

  public void turnOnIntakeRollers() {
    _intakeIO.setRollerMotorOutput(IntakeConstants.INTAKE_ACQUIRE_SPEED.get());
  }

  public void stopRollers() {
    _intakeIO.setRollerMotorOutput(0.0);
  }
}
