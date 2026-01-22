package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

/**
 * Factory class for creating shooter-related commands.
 *
 * <p>This class provides static factory methods for common shooter operations including
 * manual speed control, distance-based shooting, and flywheel idle management.
 *
 * <p><b>Key Shooting Modes:</b>
 * <ul>
 *   <li><b>Manual Speed:</b> Set a specific RPM for testing or fixed-distance shots</li>
 *   <li><b>Distance-Based:</b> Automatically calculate RPM based on distance to target</li>
 *   <li><b>Idle Speed:</b> Keep flywheel spinning slowly to reduce spin-up time</li>
 * </ul>
 *
 * <p><b>Usage Example - Distance-Based Shooting:</b>
 * <pre>
 * Command shootSequence = Commands.sequence(
 *   // 1. Spin up to correct speed for current distance
 *   ShooterCommands.setFlywheelSpeedForDistance(shooter, drive.getDistanceToHub()),
 *
 *   // 2. Wait for flywheel to reach target RPM
 *   ShooterCommands.waitForFlywheelsToReachSpeed(shooter),
 *
 *   // 3. Feed the game piece
 *   FeederCommands.feedShooter(feeder).withTimeout(1.0),
 *
 *   // 4. Stop shooter
 *   ShooterCommands.stopFlywheels(shooter)
 * );
 * </pre>
 */
public class ShooterCommands {

  /**
   * Creates a command to set the flywheel to a specific target speed.
   *
   * <p>This command continuously commands the flywheel to spin at the specified
   * speed. The motor controller's PID handles reaching and maintaining the speed.
   *
   * <p><b>Use cases:</b>
   * <ul>
   *   <li>Testing specific RPMs during characterization</li>
   *   <li>Fixed-distance shots (e.g., always shoot from same spot)</li>
   *   <li>Manual override of distance-based shooting</li>
   * </ul>
   *
   * <p><b>Example:</b>
   * <pre>
   * import static edu.wpi.first.units.Units.RPM;
   *
   * // Set to 3000 RPM
   * Command shoot = ShooterCommands.setFlywheelTargetSpeed(shooter, RPM.of(3000));
   *
   * // Bind to button
   * driver.rightBumper().whileTrue(shoot);
   * </pre>
   *
   * <p><b>Remember:</b> Use waitForFlywheelsToReachSpeed() before feeding to
   * ensure the flywheel is at the correct speed for an accurate shot.
   *
   * @param shooter The shooter subsystem
   * @param speed The target flywheel speed (use RPM.of(), RadiansPerSecond.of(), etc.)
   * @return A command that maintains the flywheel at the target speed
   */
  public static Command setFlywheelTargetSpeed(Shooter shooter, AngularVelocity speed) {
    return Commands.run(
        () -> {
          shooter.setFlywheelSpeed(speed);
        },
        shooter);
  }

  /**
   * Creates a command to stop the flywheel.
   *
   * <p>This commands the flywheel to stop spinning. Note that heavy flywheels take
   * time to spin down due to momentum - they won't stop instantly.
   *
   * <p><b>Use cases:</b>
   * <ul>
   *   <li>After completing shots to save battery</li>
   *   <li>When switching between shooting modes</li>
   *   <li>When robot is disabled (safety)</li>
   * </ul>
   *
   * <p><b>Alternative:</b> You might also use runFlywheelsAtIdle() instead of fully
   * stopping, which keeps the flywheel spinning slowly so the next shot spins up faster.
   *
   * @param shooter The shooter subsystem
   * @return A command that stops the flywheel
   */
  public static Command stopFlywheels(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.stopFlywheels();
        },
        shooter);
  }

  /**
   * Creates a command to run the flywheel at a low idle speed.
   *
   * <p>Instead of fully stopping the flywheel between shots, this keeps it spinning
   * slowly. This reduces the time needed to spin up for the next shot.
   *
   * <p><b>Trade-offs:</b>
   * <ul>
   *   <li><b>Advantage:</b> Faster response time - next shot is ready sooner</li>
   *   <li><b>Disadvantage:</b> Uses more battery power</li>
   * </ul>
   *
   * <p><b>When to use:</b>
   * <ul>
   *   <li>During teleop when you expect to shoot frequently</li>
   *   <li>While approaching the target</li>
   *   <li>As a default command if battery isn't a concern</li>
   * </ul>
   *
   * <p>The idle speed is set in ShooterConstants.SHOOTER_IDLE_DUTY_CYCLE_OUTPUT
   * and can be tuned based on your robot's needs.
   *
   * @param shooter The shooter subsystem
   * @return A command that runs the flywheel at idle speed
   */
  public static Command runFlywheelsAtIdle(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.setFlyWheelDutyCycle(ShooterConstants.SHOOTER_IDLE_DUTY_CYCLE_OUTPUT);
        },
        shooter);
  }

  /**
   * Creates a command to set flywheel speed based on distance to the target.
   *
   * <p>This is the core of distance-based shooting! It automatically calculates
   * the required flywheel RPM using the lookup table in ShooterConstants.
   *
   * <p><b>How it works:</b>
   * <ol>
   *   <li>Gets the current distance to target</li>
   *   <li>Calls shooter.getSpeedForDistance() to look up required RPM</li>
   *   <li>Lookup table interpolates between known data points</li>
   *   <li>Sets flywheel to calculated speed</li>
   *   <li>Repeats every 20ms to adjust if distance changes</li>
   * </ol>
   *
   * <p><b>Usage with vision or odometry:</b>
   * <pre>
   * // Get distance from Drive subsystem's odometry
   * Distance distanceToHub = drive.getDistanceToHub();
   *
   * // Create command
   * Command spinUp = ShooterCommands.setFlywheelSpeedForDistance(shooter, distanceToHub);
   * </pre>
   *
   * <p><b>Dynamic vs. Static distance:</b>
   * <ul>
   *   <li><b>Static:</b> Pass distance once, RPM stays constant even if robot moves</li>
   *   <li><b>Dynamic:</b> Call drive.getDistanceToHub() repeatedly to update as robot moves</li>
   * </ul>
   *
   * <p><b>Complete shooting sequence:</b>
   * <pre>
   * Command shoot = Commands.sequence(
   *   // 1. Calculate and spin to correct speed
   *   this.setFlywheelSpeedForDistance(shooter, drive.getDistanceToHub()),
   *
   *   // 2. Wait until at speed
   *   this.waitForFlywheelsToReachSpeed(shooter),
   *
   *   // 3. Feed the game piece
   *   FeederCommands.runFeeder(feeder).withTimeout(1.0)
   * );
   * </pre>
   *
   * @param shooter The shooter subsystem
   * @param distance Distance to the target (from odometry or vision)
   * @return A command that continuously calculates and sets the correct flywheel speed
   */
  public static Command setFlywheelSpeedForDistance(Shooter shooter, Distance distance) {
    return Commands.run(
        () -> {
          AngularVelocity targetSpeed = shooter.getSpeedForDistance(distance);
          shooter.setFlywheelSpeed(targetSpeed);
        },
        shooter);
  }

  /**
   * Creates a command that waits until the flywheel reaches its target speed.
   *
   * <p>This command does nothing except wait - it finishes when the flywheel is
   * within the acceptable tolerance of the target RPM.
   *
   * <p><b>Why is this important?</b>
   * If you feed game pieces before the flywheel is up to speed, shots will be:
   * <ul>
   *   <li>Weak - won't reach the target</li>
   *   <li>Inconsistent - every shot different depending on flywheel speed</li>
   *   <li>Inaccurate - trajectory depends heavily on launch speed</li>
   * </ul>
   *
   * <p><b>How it works:</b>
   * <ul>
   *   <li>Calls shooter.areFlywheelsAtTargetSpeed() every 20ms</li>
   *   <li>Returns false until flywheel is within tolerance</li>
   *   <li>Returns true when ready, allowing the sequence to continue</li>
   * </ul>
   *
   * <p><b>Usage in sequences:</b>
   * <pre>
   * Commands.sequence(
   *   ShooterCommands.setFlywheelTargetSpeed(shooter, RPM.of(3000)),  // Start spinning
   *   ShooterCommands.waitForFlywheelsToReachSpeed(shooter),          // Wait until ready
   *   FeederCommands.feedShooter(feeder)                              // Now feed!
   * );
   * </pre>
   *
   * <p><b>Timeout recommendation:</b> Consider adding a timeout in case something
   * goes wrong:
   * <pre>
   * ShooterCommands.waitForFlywheelsToReachSpeed(shooter).withTimeout(2.0)
   * </pre>
   *
   * @param shooter The shooter subsystem
   * @return A command that finishes when the flywheel reaches target speed
   */
  public static Command waitForFlywheelsToReachSpeed(Shooter shooter) {
    return Commands.waitUntil(() -> shooter.areFlywheelsAtTargetSpeed());
  }
}
