package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.HardwareConstants;

public class RealClimberIO implements ClimberIO {
  private SparkMax _climberMotor;

  public RealClimberIO() {
    configClimberMotor();
  }

  public void configClimberMotor() {
    _climberMotor = new SparkMax(HardwareConstants.CanIds.CLIMBER_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(ClimberConstants.CLIMBER_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ClimberConstants.CLIMBER_SMART_CURRENT_LIMIT)
        .secondaryCurrentLimit(ClimberConstants.CLIMBER_SECONDARY_CURRENT_LIMIT)
        .voltageCompensation(12.0);

    tryUntilOk(
        _climberMotor,
        5,
        () ->
            _climberMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void updateInputs(ClimberIOInputs inputs) {
    // |================= START CLIMBER MOTOR LOGGING =================|
    inputs._climberMotorTemperature = Celsius.of(_climberMotor.getMotorTemperature());
    inputs._climberMotorVoltage =
        Volts.of(_climberMotor.getAppliedOutput() * _climberMotor.getBusVoltage());
    inputs._climberMotorCurrent = Amps.of(_climberMotor.getOutputCurrent());
    // |================= END CLIMBER MOTOR LOGGING =================|
  }

  // |============================== MOTOR METHODS ============================== |
  public void setMotorSpeed(double speed) {
    _climberMotor.set(speed);
  }
}
