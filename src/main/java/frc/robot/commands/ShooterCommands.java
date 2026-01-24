package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

/**
 * Factory class for creating shooter-related commands.
 *
 * <p>This class provides static factory methods for common shooter operations including manual
 * speed control, distance-based shooting, and flywheel idle management.
 *
 * <p><b>Key Shooting Modes:</b>
 *
 * <ul>
 *   <li><b>Manual Speed:</b> Set a specific RPM for testing or fixed-distance shots
 *   <li><b>Distance-Based:</b> Automatically calculate RPM based on distance to target
 *   <li><b>Idle Speed:</b> Keep flywheel spinning slowly to reduce spin-up time
 * </ul>
 *
 * <p><b>Usage Example - Distance-Based Shooting:</b>
 *
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

  // Feedforward characterization tuning constants
  private static final double FF_START_DELAY = 2.0; // seconds to wait for idle before ramp
  private static final double FF_RAMP_RATE = 0.1; // Volts per second

  /**
   * Creates a command to set the flywheel to a specific target speed.
   *
   * <p>This command sets the flywheel target speed once and finishes immediately. The motor
   * controller's PID continues to maintain the speed after the command completes.
   *
   * <p><b>Use cases:</b>
   *
   * <ul>
   *   <li>Testing specific RPMs during characterization
   *   <li>Fixed-distance shots (e.g., always shoot from same spot)
   *   <li>Manual override of distance-based shooting
   * </ul>
   *
   * <p><b>Example:</b>
   *
   * <pre>
   * import static edu.wpi.first.units.Units.RPM;
   *
   * // Set to 3000 RPM once, then flywheel maintains that speed
   * Command shoot = ShooterCommands.setFlywheelTargetSpeed(shooter, RPM.of(3000));
   *
   * // Bind to button - sets speed on press, doesn't need to be held
   * driver.rightBumper().onTrue(shoot);
   * </pre>
   *
   * <p><b>Remember:</b> Use waitForFlywheelsToReachSpeed() before feeding to ensure the flywheel is
   * at the correct speed for an accurate shot.
   *
   * @param shooter The shooter subsystem
   * @param speed The target flywheel speed (use RPM.of(), RadiansPerSecond.of(), etc.)
   * @return A command that sets the flywheel target speed once and finishes immediately
   */
  public static Command setFlywheelTargetSpeed(Shooter shooter, AngularVelocity speed) {
    return Commands.runOnce(
        () -> {
          shooter.setFlywheelSpeed(speed);
        },
        shooter);
  }

  /**
   * Creates a command to stop the flywheel.
   *
   * <p>This command stops the flywheel once and finishes immediately. Note that heavy flywheels
   * take time to spin down due to momentum - they won't stop instantly, but the command completes
   * right away after sending the stop signal.
   *
   * <p><b>Use cases:</b>
   *
   * <ul>
   *   <li>After completing shots to save battery
   *   <li>When switching between shooting modes
   *   <li>When robot is disabled (safety)
   * </ul>
   *
   * <p><b>Alternative:</b> You might also use runFlywheelsAtIdle() instead of fully stopping, which
   * keeps the flywheel spinning slowly so the next shot spins up faster.
   *
   * @param shooter The shooter subsystem
   * @return A command that stops the flywheel once and finishes immediately
   */
  public static Command stopFlywheels(Shooter shooter) {
    return Commands.runOnce(
        () -> {
          shooter.stopFlywheels();
        },
        shooter);
  }

  /**
   * Creates a command to run the flywheel at a low idle speed.
   *
   * <p>This command sets the flywheel to idle speed once and finishes immediately. The flywheel
   * continues spinning at idle speed after the command completes. Instead of fully stopping the
   * flywheel between shots, this keeps it spinning slowly, which reduces the time needed to spin up
   * for the next shot.
   *
   * <p><b>Trade-offs:</b>
   *
   * <ul>
   *   <li><b>Advantage:</b> Faster response time - next shot is ready sooner
   *   <li><b>Disadvantage:</b> Uses more battery power
   * </ul>
   *
   * <p><b>When to use:</b>
   *
   * <ul>
   *   <li>During teleop when you expect to shoot frequently
   *   <li>While approaching the target
   *   <li>As a default command if battery isn't a concern
   * </ul>
   *
   * <p>The idle speed is set in ShooterConstants.SHOOTER_IDLE_DUTY_CYCLE_OUTPUT and can be tuned
   * based on your robot's needs.
   *
   * @param shooter The shooter subsystem
   * @return A command that sets the flywheel to idle speed once and finishes immediately
   */
  public static Command runFlywheelsAtIdle(Shooter shooter) {
    return Commands.runOnce(
        () -> {
          shooter.setFlyWheelDutyCycle(ShooterConstants.SHOOTER_IDLE_DUTY_CYCLE_OUTPUT);
        },
        shooter);
  }

  /**
   * Creates a command to set flywheel speed based on distance to the target.
   *
   * <p>This is the core of distance-based shooting! It automatically calculates the required
   * flywheel RPM using the lookup table in ShooterConstants. This command calculates the speed once
   * and finishes immediately.
   *
   * <p><b>How it works:</b>
   *
   * <ol>
   *   <li>Gets the current distance to target
   *   <li>Calls shooter.getSpeedForDistance() to look up required RPM
   *   <li>Lookup table interpolates between known data points
   *   <li>Sets flywheel to calculated speed
   *   <li>Command finishes - motor controller maintains the speed
   * </ol>
   *
   * <p><b>Usage with vision or odometry:</b>
   *
   * <pre>
   * // Get distance from Drive subsystem's odometry
   * Distance distanceToHub = drive.getDistanceToHub();
   *
   * // Create command - sets speed once based on current distance
   * Command spinUp = ShooterCommands.setFlywheelSpeedForDistance(shooter, distanceToHub);
   * </pre>
   *
   * <p><b>Important:</b> This command calculates speed based on the distance at the moment the
   * command runs. If the robot moves after the command finishes, the speed won't update. For
   * continuously updating speed as the robot moves, wrap this in a Commands.run() or call it
   * repeatedly.
   *
   * <p><b>Complete shooting sequence:</b>
   *
   * <pre>
   * Command shoot = Commands.sequence(
   *   // 1. Calculate and set speed for current distance
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
   * @return A command that calculates and sets the correct flywheel speed once, then finishes
   */
  public static Command setFlywheelSpeedForDistance(Shooter shooter, Distance distance) {
    return Commands.runOnce(
        () -> {
          AngularVelocity targetSpeed = shooter.getSpeedForDistance(distance);
          shooter.setFlywheelSpeed(targetSpeed);
        },
        shooter);
  }

  /**
   * Creates a command that waits until the flywheel reaches its target speed.
   *
   * <p>This command does nothing except wait - it finishes when the flywheel is within the
   * acceptable tolerance of the target RPM.
   *
   * <p><b>Why is this important?</b> If you feed game pieces before the flywheel is up to speed,
   * shots will be:
   *
   * <ul>
   *   <li>Weak - won't reach the target
   *   <li>Inconsistent - every shot different depending on flywheel speed
   *   <li>Inaccurate - trajectory depends heavily on launch speed
   * </ul>
   *
   * <p><b>How it works:</b>
   *
   * <ul>
   *   <li>Calls shooter.areFlywheelsAtTargetSpeed() every 20ms
   *   <li>Returns false until flywheel is within tolerance
   *   <li>Returns true when ready, allowing the sequence to continue
   * </ul>
   *
   * <p><b>Usage in sequences:</b>
   *
   * <pre>
   * Commands.sequence(
   *   ShooterCommands.setFlywheelTargetSpeed(shooter, RPM.of(3000)),  // Start spinning
   *   ShooterCommands.waitForFlywheelsToReachSpeed(shooter),          // Wait until ready
   *   FeederCommands.feedShooter(feeder)                              // Now feed!
   * );
   * </pre>
   *
   * <p><b>Timeout recommendation:</b> Consider adding a timeout in case something goes wrong:
   *
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

  /**
   * Simple feedforward characterization for the shooter flywheel.
   *
   * <p>Ramps voltage up linearly while sampling the resulting angular velocity. When the command is
   * canceled, performs a least-squares fit to compute kS and kV (volts and volts per RPM
   * respectively) and prints the results.
   *
   * <p>Use this in voltage-control mode only. Example: hold the X button to run the
   * characterization, release to finish and print results.
   *
   * <p><b>Units:</b> kS is in Volts, kV is in Volts per RPM
   */
  public static Command feedforwardCharacterization(Shooter shooter) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Ensure idle for a moment so things settle
        Commands.run(() -> shooter.runCharacterization(Volts.of(0.0)), shooter)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Ramp voltage and sample
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  shooter.runCharacterization(Volts.of(voltage));
                  // Sample velocity in rad/s, convert to RPM for regression
                  double velocityRadPerSec = shooter.getFFCharacterizationVelocity();
                  double velocityRPM = velocityRadPerSec * 60.0 / (2.0 * Math.PI);
                  velocitySamples.add(velocityRPM);
                  voltageSamples.add(voltage);
                },
                shooter)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Shooter FF Characterization Results **********");
                  System.out.println("\tkS (Volts): " + formatter.format(kS));
                  System.out.println("\tkV (Volts per RPM): " + formatter.format(kV));
                }));
  }
}
