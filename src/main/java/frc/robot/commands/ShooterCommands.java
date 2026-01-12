package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
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
}
