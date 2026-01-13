package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.feeder.Feeder;
import java.util.function.DoubleSupplier;

public class FeederCommands {
  private static final double DEADBAND = 0.1;

  /**
   * Command to run the feeder motor using joystick input. Applies deadband to joystick values for
   * smooth control.
   *
   * @param feeder The feeder subsystem
   * @param speedSupplier Joystick value supplier (-1.0 to 1.0)
   * @return Command that continuously runs the feeder based on joystick input
   */
  public static Command runFeederWithJoystick(Feeder feeder, DoubleSupplier speedSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband to joystick input (similar to DriveCommands)
          double speed = MathUtil.applyDeadband(speedSupplier.getAsDouble(), DEADBAND);

          // Set the motor speed (duty cycle from -1.0 to 1.0)
          feeder.setFeederSpeed(speed);
        },
        feeder);
  }

  /**
   * Command to run the feeder at a fixed % output.
   *
   * @param feeder The feeder subsystem
   * @param speed Fixed speed from -1.0 to 1.0
   * @return Command that runs the feeder at the specified speed
   */
  public static Command runFeederAtPercentOutput(Feeder feeder, double speed) {
    return Commands.run(
        () -> {
          feeder.setFeederSpeed(speed);
        },
        feeder);
  }

  /**
   * Command to stop the feeder motor.
   *
   * @param feeder The feeder subsystem
   * @return Command that stops the feeder
   */
  public static Command stopFeeder(Feeder feeder) {
    return Commands.runOnce(() -> feeder.stop(), feeder);
  }
}
