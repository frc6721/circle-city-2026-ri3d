// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO _shooterIO;
  private final ShooterIOInputsAutoLogged _shooterInputs = new ShooterIOInputsAutoLogged();
  private AngularVelocity _targetFlywheelSpeed;

  /** Creates a new Shooter. */
  public Shooter(ShooterIO shooterIO) {
    this._shooterIO = shooterIO;
    this.stopFlywheels();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    _shooterIO.updateInputs(_shooterInputs);
    Logger.processInputs("Shooter", _shooterInputs);

    // LOGGING
    Logger.recordOutput(
        "Shooter/Current-Flywheel-Speed",
        _shooterInputs._flywheelMotorVelocity.in(RadiansPerSecond));
    Logger.recordOutput(
        "Shooter/Current-Flywheel-Speed-RPM",
        _shooterInputs._flywheelMotorVelocity.in(RevolutionsPerSecond) * 60);

    Logger.recordOutput(
        "Shooter/Desired-Flywheel-Speed", _targetFlywheelSpeed.in(RadiansPerSecond));
    Logger.recordOutput(
        "Shooter/Desired-Flywheel-Speed-RPM", _targetFlywheelSpeed.in(RevolutionsPerSecond) * 60);
  }

  public void stopFlywheels() {
    _targetFlywheelSpeed = RadiansPerSecond.of(0);
    _shooterIO.stopFlywheel();
  }

  public void setFlywheelSpeed(AngularVelocity speed) {
    _targetFlywheelSpeed = speed;
    _shooterIO.setFlywheelSpeed(speed);
  }

  /**
   * Calculates the required flywheel speed for a given distance to target. Uses the lookup table
   * from ShooterConstants with interpolation.
   *
   * @param distance Distance to the target
   * @return Required flywheel speed
   */
  public AngularVelocity getSpeedForDistance(Distance distance) {
    // Convert distance to meters for lookup table
    double distanceMeters = distance.in(Meters);

    // Get interpolated speed from lookup table (returns RPM)
    double speedRPM = ShooterConstants.getSpeedForDistance(distanceMeters);

    // Log the calculation for debugging
    Logger.recordOutput("Shooter/CalculatedDistance_meters", distanceMeters);
    Logger.recordOutput("Shooter/CalculatedSpeed_RPM", speedRPM);

    // Convert RPM to AngularVelocity and return
    return RPM.of(speedRPM);
  }

  private boolean areFlywheelsAtTargetSpeed() {
    return Math.abs(
            _targetFlywheelSpeed.in(RadiansPerSecond)
                - _shooterInputs._flywheelMotorVelocity.in(RadiansPerSecond))
        <= Math.abs(
            _targetFlywheelSpeed.in(RadiansPerSecond) * ShooterConstants.FLYWHEEL_PID_TOLERANCE);
    // the tolerance is a percent error of the target speed we are allowed
  }
}
