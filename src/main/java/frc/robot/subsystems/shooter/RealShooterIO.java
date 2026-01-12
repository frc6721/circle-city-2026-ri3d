package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.HardwareConstants;

public class RealShooterIO implements ShooterIO {
  private SparkMax _flywheelMotor;

  public RealShooterIO() {
    configFlywheelMotor();
  }

  public void configFlywheelMotor() {
    _flywheelMotor = new SparkMax(HardwareConstants.CanIds.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(ShooterConstants.SHOOTER_FLYWHEEL_INVERTED)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.SHOOTER_FLYWHEEL_SMART_CURRENT_LIMIT)
        .secondaryCurrentLimit(ShooterConstants.SHOOTER_FLYWHEEL_SECONDARY_CURRENT_LIMIT)
        .voltageCompensation(12.0);

    config
        .encoder
        .positionConversionFactor(
            2 * Math.PI / ShooterConstants.FLYWHEEL_GEAR_RATIO) // motor rotations to flywheel rad
        .velocityConversionFactor(
            (2 * Math.PI)
                / 60.0
                / ShooterConstants.FLYWHEEL_GEAR_RATIO) // motor RPM to flywheel rad/s
        .inverted(false);
    config.closedLoop.pidf(
        ShooterConstants.SHOOTER_FLYWHEEL_PID_KP.get(),
        ShooterConstants.SHOOTER_FLYWHEEL_PID_KI.get(),
        ShooterConstants.SHOOTER_FLYWHEEL_PID_KD.get(),
        ShooterConstants.SHOOTER_FLYWHEEL_PID_FF.get());

    tryUntilOk(
        _flywheelMotor,
        5,
        () ->
            _flywheelMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void updateInputs(ShooterIOInputs inputs) {
    // |================= START SHOOTER FLYWHEEL MOTOR LOGGING =================|
    inputs._flywheelMotorTemperature = Celsius.of(_flywheelMotor.getMotorTemperature());
    inputs._flywheelMotorVelocity = RadiansPerSecond.of(_flywheelMotor.getEncoder().getVelocity());
    inputs._flywheelMotorVoltage =
        Volts.of(_flywheelMotor.getAppliedOutput() * _flywheelMotor.getBusVoltage());
    inputs._flywheelMotorCurrent = Amps.of(_flywheelMotor.getOutputCurrent());
    // |================= END SHOOTER FLYWHEEL MOTOR LOGGING =================|

  }

  // |============================== FLYWHEEL MOTOR METHODS ============================== |
  public void setFlywheelSpeed(AngularVelocity speed) {
    _flywheelMotor
        .getClosedLoopController()
        .setReference(speed.in(RadiansPerSecond), ControlType.kVelocity);
  }

  public void stopFlywheel() {
    // reset the integral accumulator to prevent integral windup
    _flywheelMotor.getClosedLoopController().setIAccum(0);
    _flywheelMotor.stopMotor();
  }
}
