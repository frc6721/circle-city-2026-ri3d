package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.feeder.Feeder;
import java.util.function.DoubleSupplier;

/**
 * Factory class for creating feeder-related commands.
 *
 * <p>The feeder commands are simple since the feeder subsystem doesn't require complex
 * control - it either feeds game pieces, stops, or runs at a specific speed.
 *
 * <p><b>Typical Usage Pattern:</b>
 * The feeder is usually controlled automatically as part of a shooting sequence:
 * <ol>
 *   <li>Shooter spins up to target RPM</li>
 *   <li>Wait for shooter to reach speed</li>
 *   <li>Run feeder to push game piece into shooter</li>
 *   <li>Stop feeder after shot</li>
 * </ol>
 *
 * <p><b>Example Shooting Sequence:</b>
 * <pre>
 * Command shootSequence = Commands.sequence(
 *   ShooterCommands.setFlywheelTargetSpeed(shooter, RPM.of(3000)),
 *   ShooterCommands.waitForFlywheelsToReachSpeed(shooter),
 *   FeederCommands.runFeederAtPercentOutput(feeder, 0.8).withTimeout(1.0),
 *   FeederCommands.stopFeeder(feeder)
 * );
 * </pre>
 */
public class FeederCommands {
  /** Deadband threshold for joystick input - values below this are treated as zero. */
  private static final double DEADBAND = 0.1;

  /**
   * Creates a command to run the feeder using joystick input for manual control.
   *
   * <p>This applies a deadband to the joystick input to prevent drift from causing
   * unwanted movement. Small joystick values near zero are ignored.
   *
   * <p><b>When to use:</b>
   * <ul>
   *   <li>Manual testing during practice</li>
   *   <li>Troubleshooting feed issues</li>
   *   <li>Fine control during setup</li>
   * </ul>
   *
   * <p><b>Example - bind to joystick axis:</b>
   * <pre>
   * // Continuous control with left Y axis
   * feeder.setDefaultCommand(
   *   FeederCommands.runFeederWithJoystick(feeder, () -> -driver.getLeftY())
   * );
   * </pre>
   *
   * <p><b>Deadband:</b> Values between -0.1 and +0.1 are treated as 0 to prevent
   * drift when the joystick isn't perfectly centered.
   *
   * @param feeder The feeder subsystem
   * @param speedSupplier Joystick value supplier, typically from a controller axis (-1.0 to 1.0)
   * @return A command that continuously runs the feeder based on joystick input
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
   * Creates a command to run the feeder at a fixed speed (duty cycle output).
   *
   * <p>This is the most common way to control the feeder - run at a constant speed
   * for a set duration. Typically used in shooting sequences.
   *
   * <p><b>Speed values:</b>
   * <ul>
   *   <li>+1.0 = Full power forward (feed into shooter)</li>
   *   <li>+0.7 = 70% power (typical for controlled feeding)</li>
   *   <li>0.0 = Stopped</li>
   *   <li>-0.5 = Half power reverse (clear jam or pull back)</li>
   * </ul>
   *
   * <p><b>Example - Feed for 1 second:</b>
   * <pre>
   * Command feed = FeederCommands.runFeederAtPercentOutput(feeder, 0.8)
   *   .withTimeout(1.0);
   * </pre>
   *
   * <p><b>Example - Feed until sensor detects game piece left:</b>
   * <pre>
   * Command feed = FeederCommands.runFeederAtPercentOutput(feeder, 0.8)
   *   .until(() -> !sensor.hasGamePiece());
   * </pre>
   *
   * <p><b>Tuning tip:</b> Start with lower speeds (0.5-0.7) and increase if needed.
   * Too fast can cause game pieces to bounce or jam.
   *
   * @param feeder The feeder subsystem
   * @param speed Fixed duty cycle from -1.0 (full reverse) to +1.0 (full forward)
   * @return A command that runs the feeder at the specified speed
   */
  public static Command runFeederAtPercentOutput(Feeder feeder, double speed) {
    return Commands.run(
        () -> {
          feeder.setFeederSpeed(speed);
        },
        feeder);
  }

  /**
   * Creates a command to stop the feeder motor.
   *
   * <p>This command runs once to stop the feeder, then immediately finishes.
   * Unlike the continuously-running commands above, this uses runOnce().
   *
   * <p><b>When to use:</b>
   * <ul>
   *   <li>At the end of shooting sequences</li>
   *   <li>When aborting a shot</li>
   *   <li>Emergency stop</li>
   *   <li>After manually clearing a jam</li>
   * </ul>
   *
   * <p><b>Example in sequence:</b>
   * <pre>
   * Commands.sequence(
   *   FeederCommands.runFeederAtPercentOutput(feeder, 0.8).withTimeout(1.0),
   *   FeederCommands.stopFeeder(feeder)  // Ensure it stops after timeout
   * );
   * </pre>
   *
   * <p><b>Note:</b> Using runOnce() means this command executes once and finishes
   * immediately, unlike run() which would continue indefinitely. This is appropriate
   * for stopping because we just need to send the stop command once.
   *
   * @param feeder The feeder subsystem
   * @return A command that stops the feeder once and finishes
   */
  public static Command stopFeeder(Feeder feeder) {
    return Commands.runOnce(() -> feeder.stop(), feeder);
  }
}
