package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Visualizes the shooter flywheel mechanism using Mechanism2d and logs a Pose3d for AdvantageScope
 * 3D visualization.
 *
 * <p>The visualization shows:
 *
 * <ul>
 *   <li><b>Spinning octagon:</b> Represents the flywheel rotating based on velocity
 *   <li><b>Green background:</b> When flywheel is at target speed
 *   <li><b>Black background:</b> When flywheel is spinning up or idle
 * </ul>
 */
public class ShooterVisualizer {

  // Mechanism2d canvas dimensions
  private static final double WIDTH = 0.5;
  private static final double HEIGHT = 0.5;

  // Flywheel visualization radius
  private static final double FLYWHEEL_RADIUS = 0.2;
  private static final double SPOKE_LENGTH = 0.15307; // Creates an octagon shape

  // Visualization components
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d roller;

  // Configuration
  private final String name;
  private final Pose3d baseOffset;

  // State tracking for rotation animation
  private double accumulatedAngle = 0.0;
  private long lastUpdateTime = System.currentTimeMillis();

  /**
   * Creates a new ShooterVisualizer.
   *
   * @param name The name for logging (e.g., "Shooter")
   */
  public ShooterVisualizer(String name) {
    this.name = name;
    this.baseOffset =
        new Pose3d(ShooterConstants.VISUALIZATION_OFFSET, ShooterConstants.VISUALIZATION_ROTATION);

    // Create the Mechanism2d canvas
    mechanism = new LoggedMechanism2d(WIDTH, HEIGHT, new Color8Bit(Color.kBlack));

    // Create the root at center of canvas
    LoggedMechanismRoot2d root = mechanism.getRoot(name + "_PivotPoint", WIDTH / 2.0, HEIGHT / 2.0);

    // Create the main roller spoke (this will rotate)
    roller =
        root.append(
            new LoggedMechanismLigament2d(
                name + "_Roller", FLYWHEEL_RADIUS, 0, 0, new Color8Bit(Color.kAliceBlue)));

    // Create an octagon shape by chaining ligaments
    // Each turn is 45 degrees (360/8) to form an octagon
    LoggedMechanismLigament2d side1 =
        roller.append(
            new LoggedMechanismLigament2d(
                name + "_Side1", SPOKE_LENGTH, 112.5, 6, new Color8Bit(Color.kAliceBlue)));

    LoggedMechanismLigament2d side2 =
        side1.append(
            new LoggedMechanismLigament2d(
                name + "_Side2", SPOKE_LENGTH, 45, 6, new Color8Bit(Color.kAliceBlue)));

    LoggedMechanismLigament2d side3 =
        side2.append(
            new LoggedMechanismLigament2d(
                name + "_Side3", SPOKE_LENGTH, 45, 6, new Color8Bit(Color.kAliceBlue)));

    LoggedMechanismLigament2d side4 =
        side3.append(
            new LoggedMechanismLigament2d(
                name + "_Side4", SPOKE_LENGTH, 45, 6, new Color8Bit(Color.kAliceBlue)));

    LoggedMechanismLigament2d side5 =
        side4.append(
            new LoggedMechanismLigament2d(
                name + "_Side5", SPOKE_LENGTH, 45, 6, new Color8Bit(Color.kAliceBlue)));

    LoggedMechanismLigament2d side6 =
        side5.append(
            new LoggedMechanismLigament2d(
                name + "_Side6", SPOKE_LENGTH, 45, 6, new Color8Bit(Color.kAliceBlue)));

    LoggedMechanismLigament2d side7 =
        side6.append(
            new LoggedMechanismLigament2d(
                name + "_Side7", SPOKE_LENGTH, 45, 6, new Color8Bit(Color.kAliceBlue)));

    side7.append(
        new LoggedMechanismLigament2d(
            name + "_Side8", SPOKE_LENGTH, 45, 6, new Color8Bit(Color.kAliceBlue)));
  }

  /**
   * Updates the visualizer with the current state.
   *
   * @param currentVelocity The current flywheel angular velocity
   * @param targetVelocity The target flywheel angular velocity
   * @param atTargetSpeed Whether the flywheel is at the target speed
   */
  public void update(
      AngularVelocity currentVelocity, AngularVelocity targetVelocity, boolean atTargetSpeed) {

    // Calculate time delta for smooth rotation animation
    long currentTime = System.currentTimeMillis();
    double deltaTimeSeconds = (currentTime - lastUpdateTime) / 1000.0;
    lastUpdateTime = currentTime;

    // Update accumulated angle based on velocity (scaled for visualization)
    // Divide by 10 to slow down the visualization for better visibility
    double velocityDegreesPerSecond = Math.toDegrees(currentVelocity.in(RadiansPerSecond)) / 10.0;
    accumulatedAngle += velocityDegreesPerSecond * deltaTimeSeconds;

    // Keep angle in 0-360 range
    accumulatedAngle = accumulatedAngle % 360.0;

    // Update the roller angle to show rotation
    roller.setAngle(accumulatedAngle);

    // Update background color based on at-target status
    if (atTargetSpeed && targetVelocity.in(RadiansPerSecond) > 0) {
      mechanism.setBackgroundColor(new Color8Bit(Color.kDarkGreen));
    } else {
      mechanism.setBackgroundColor(new Color8Bit(Color.kBlack));
    }

    // Publish to SmartDashboard for Glass/AdvantageScope Mechanism2d view
    SmartDashboard.putData(name + " Visualizer", mechanism);
    Logger.recordOutput(name + "/Mechanism2d", mechanism);

    // Log the 3D pose for AdvantageScope 3D visualization
    // The rotation is around the X axis (roll) since the flywheel spins on its axis
    Pose3d pose3d =
        baseOffset.rotateBy(
            new Rotation3d(
                Degrees.of(accumulatedAngle).plus(Radians.of(baseOffset.getRotation().getX())),
                Radians.of(baseOffset.getRotation().getY()),
                Radians.of(baseOffset.getRotation().getZ())));

    Logger.recordOutput(name + "/Pose3d", pose3d);
    Logger.recordOutput(name + "/CurrentVelocityRadPerSec", currentVelocity.in(RadiansPerSecond));
    Logger.recordOutput(name + "/TargetVelocityRadPerSec", targetVelocity.in(RadiansPerSecond));
    Logger.recordOutput(name + "/AtTargetSpeed", atTargetSpeed);
    Logger.recordOutput(name + "/VisualizationAngleDegrees", accumulatedAngle);
  }

  /**
   * Simplified update method for when only velocity is available.
   *
   * @param currentVelocity The current flywheel angular velocity
   */
  public void update(AngularVelocity currentVelocity) {
    update(currentVelocity, RadiansPerSecond.zero(), false);
  }
}
