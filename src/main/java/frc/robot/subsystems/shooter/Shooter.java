// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final ShooterIO _shooterIO;
  private final ShooterIOInputsAutoLogged _shooterInputs = new ShooterIOInputsAutoLogged();
  private AngularVelocity _targetFlywheelSpeed;

  /** Creates a new Shooter. */
  public Shooter(ShooterIO shooterIO) {
    this._shooterIO = shooterIO;
    _targetFlywheelSpeed = RadiansPerSecond.of(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    _shooterIO.updateInputs(_shooterInputs);
    Logger.processInputs("Shooter", _shooterInputs);

    
    // LOGGING
    Logger.recordOutput("Shooter/Current-Flywheel-Speed", _shooterInputs._flywheelMotorVelocity.in(RadiansPerSecond));
    Logger.recordOutput("Shooter/Current-Flywheel-Speed-RPM", _shooterInputs._flywheelMotorVelocity.in(RevolutionsPerSecond)*60);

    Logger.recordOutput("Shooter/Desired-Flywheel-Speed", _targetFlywheelSpeed.in(RadiansPerSecond));
    Logger.recordOutput("Shooter/Desired-Flywheel-Speed-RPM", _targetFlywheelSpeed.in(RevolutionsPerSecond)*60);

  }

  public void stopFlywheels() {
    _shooterIO.stopFlywheels();
  }

  public void setFlywheelSpeed(AngularVelocity speed) {
    _targetFlywheelSpeed = speed;
    _shooterIO.setFlywheelSpeed(speed);
  }

  
  // TODO: THIS SHOULD BE SET AS MATH.ABS() ONCE SHOOTER FLYWHEEL PIDS ARE FIXED
  private boolean areFlywheelsAtTargetSpeed() {
    return Math.abs(_targetFlywheelSpeed.in(RadiansPerSecond) - _shooterInputs._flywheelMotorVelocity) <= 
           Math.abs(_targetFlywheelSpeed.in(RadiansPerSecond) - _targetFlywheelSpeed.in(RadiansPerSecond) * ShooterConstants.FLYWHEEL_SPEED_DEADBAND);
             // the deadband is a percent, so subtract the deadband percentage times the target from the
             // target to get the lower bound.
}

//how set speed, units of rotation, atTargetSpeed method???



}

