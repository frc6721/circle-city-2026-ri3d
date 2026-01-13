package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class ShooterCommands {

  public ShooterCommands() {}

  public static Command setFlywheelTargetSpeed(Shooter shooter, AngularVelocity speed) {
    return Commands.run(
        () -> {
          shooter.setFlywheelSpeed(speed);
        },
        shooter);
  }

  public static Command stopFlywheels(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.stopFlywheels();
        },
        shooter);
  }

  /**
   * Sets the flywheels to an idle speed defined in ShooterConstants.
   *
   * @param shooter
   * @return
   */
  public static Command runFlywheelsAtIdle(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.setFlyWheelDutyCycle(ShooterConstants.SHOOTER_IDLE_DUTY_CYCLE_OUTPUT);
        },
        shooter);
  }

  /**
   * Sets the flywheel speed based on distance to target using the lookup table. The speed is
   * automatically calculated from ShooterConstants interpolation.
   *
   * @param shooter The shooter subsystem
   * @param distance Distance to the target
   * @return Command that continuously sets the flywheel speed
   */
  public static Command setFlywheelSpeedForDistance(Shooter shooter, Distance distance) {
    return Commands.run(
        () -> {
          AngularVelocity targetSpeed = shooter.getSpeedForDistance(distance);
          shooter.setFlywheelSpeed(targetSpeed);
        },
        shooter);
  }

  public static Command waitForFlywheelsToReachSpeed(Shooter shooter) {
    return Commands.waitUntil(() -> shooter.areFlywheelsAtTargetSpeed());
  }
}
