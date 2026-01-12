package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommands {

  public ShooterCommands() {}

  public Command setFlywheelTargetSpeed(Shooter shooter, AngularVelocity speed) {
    return Commands.run(
        () -> {
          shooter.setFlywheelSpeed(speed);
        },
        shooter);
  }

  public Command stopFlywheels(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.stopFlywheels();
        },
        shooter);
  }

  /**
   * Sets the flywheel speed based on distance to target using the lookup table.
   * The speed is automatically calculated from ShooterConstants interpolation.
   * 
   * @param shooter The shooter subsystem
   * @param distance Distance to the target
   * @return Command that continuously sets the flywheel speed
   */
  public Command setFlywheelSpeedForDistance(Shooter shooter, Distance distance) {
    return Commands.run(
        () -> {
          AngularVelocity targetSpeed = shooter.getSpeedForDistance(distance);
          shooter.setFlywheelSpeed(targetSpeed);
        },
        shooter);
  }
}
