package frc.robot.subsystems.feeder;

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

public class RealFeederIO implements FeederIO {
  private SparkMax _feederMotor;

  public RealFeederIO() {
    configFeederMotor();
  }

  public void configFeederMotor() {
    // TODO: Update CAN ID in HardwareConstants
    _feederMotor = new SparkMax(HardwareConstants.CanIds.FEEDER_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(FeederConstants.FEEDER_MOTOR_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(FeederConstants.FEEDER_MOTOR_SMART_CURRENT_LIMIT)
        .secondaryCurrentLimit(FeederConstants.FEEDER_MOTOR_SECONDARY_CURRENT_LIMIT)
        .voltageCompensation(12.0);

    tryUntilOk(
        _feederMotor,
        5,
        () ->
            _feederMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    // |================= START FEEDER MOTOR LOGGING =================|
    inputs._feederMotorTemperature = Celsius.of(_feederMotor.getMotorTemperature());
    inputs._feederMotorVoltage =
        Volts.of(_feederMotor.getAppliedOutput() * _feederMotor.getBusVoltage());
    inputs._feederMotorCurrent = Amps.of(_feederMotor.getOutputCurrent());
    // |================= END FEEDER MOTOR LOGGING =================|
  }

  @Override
  public void setMotorSpeed(double speed) {
    _feederMotor.set(speed);
  }
}
