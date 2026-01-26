package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.Logger;

/**
 * Robot physical dimensions for FuelSim and collision detection.
 *
 * <p>These constants define the overall size of the robot including bumpers. They are used by
 * FuelSim to detect collisions between the robot and game pieces.
 *
 * <p><b>Robot Dimensions:</b>
 *
 * <ul>
 *   <li>Frame: 26" × 26" (square frame)
 *   <li>Bumpers: 4" thick on all sides
 *   <li>Total: 34" × 34" with bumpers (0.8636 m)
 *   <li>Bumper height: 6" off ground at top
 * </ul>
 */
public final class Dimensions {

  // Private constructor prevents instantiation (utility class)
  private Dimensions() {}

  /**
   * Full robot width including bumpers (left to right, along Y-axis).
   *
   * <p>Calculation:
   *
   * <ul>
   *   <li>Robot frame: 26"
   *   <li>Bumpers on each side: 4" × 2 = 8"
   *   <li>Total: 26" + 8" = 34" = 0.8636 m
   * </ul>
   */
  public static final Distance ROBOT_WIDTH = Inches.of(34.0);

  /**
   * Full robot length including bumpers (front to back, along X-axis).
   *
   * <p>Calculation is same as width since the robot has a square frame.
   *
   * <ul>
   *   <li>Robot frame: 26"
   *   <li>Bumpers front and back: 4" × 2 = 8"
   *   <li>Total: 26" + 8" = 34" = 0.8636 m
   * </ul>
   */
  public static final Distance ROBOT_LENGTH = Inches.of(34.0);

  /**
   * Height from ground to top of bumpers.
   *
   * <p>Bumpers are mounted in the bumper zone with the top at approximately 6" off the ground. This
   * is used by FuelSim to determine if game pieces can be pushed by the robot (pieces above bumper
   * height won't collide with the robot body).
   */
  public static final Distance BUMPER_HEIGHT = Inches.of(6.0);

  // Log dimensions at class load
  static {
    Logger.recordOutput("Constants/Dimensions/RobotWidth_m", ROBOT_WIDTH.in(Meters));
    Logger.recordOutput("Constants/Dimensions/RobotLength_m", ROBOT_LENGTH.in(Meters));
    Logger.recordOutput("Constants/Dimensions/BumperHeight_m", BUMPER_HEIGHT.in(Meters));
    Logger.recordOutput("Constants/Dimensions/RobotWidth_in", ROBOT_WIDTH.in(Inches));
    Logger.recordOutput("Constants/Dimensions/RobotLength_in", ROBOT_LENGTH.in(Inches));
    Logger.recordOutput("Constants/Dimensions/BumperHeight_in", BUMPER_HEIGHT.in(Inches));
  }
}
