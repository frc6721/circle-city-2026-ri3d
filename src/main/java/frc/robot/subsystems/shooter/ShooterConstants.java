package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.units.measure.AngularVelocity;

import java.util.TreeMap;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import java.util.Map;

public class ShooterConstants {
  /************************
   *
   * SHOOTER FLYWHEEL PID CONSTANTS
   *
   *************************/
  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_PID_KP =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/kP", 0.000);

  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_PID_KI =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/kI", 0.0000);
  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_PID_KD =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/kD", 0.0000);
  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_PID_FF =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/kFF", 0.00022);

  /***********************
   *
   * SHOOTER SPEED SETPOINTS
   *
   * units: RPM
   *
   **********************/
  public static final LoggedNetworkNumber FLYWHEEL_SPEED =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_SPEED", 1500);

  /***************
   *
   * SHOOTER MECHANICAL CONSTANTS
   *
   ***************/
  public static final double FLYWHEEL_GEAR_RATIO = 2;

  /******************
   * SHOOTER SOFTWARE CONSTANTS
   *
   *****************/
  public static final double FLYWHEEL_PID_TOLERANCE = 0.05;

  public static final int SHOOTER_FLYWHEEL_SMART_CURRENT_LIMIT = 100;
  public static final double SHOOTER_FLYWHEEL_SECONDARY_CURRENT_LIMIT = 100;
  public static final boolean SHOOTER_FLYWHEEL_INVERTED = false;
  public static final AngularVelocity MIN_FLYWHEEL_SPEED = RevolutionsPerSecond.of(100 / 60.0); // 100 RPM
  public static final AngularVelocity MAX_FLYWHEEL_SPEED = RevolutionsPerSecond.of(2000 / 60.0); // 5000 RPM

  /***********************
   *
   * DISTANCE TO SPEED LOOKUP TABLE
   *
   * Distance units: meters
   * Speed units: RPM
   *
   **********************/
  // Data point 1
  public static final LoggedNetworkNumber LOOKUP_DISTANCE_1 =
      new LoggedNetworkNumber("Shooter/Lookup/Distance1_meters", 1.0);
  public static final LoggedNetworkNumber LOOKUP_SPEED_1 =
      new LoggedNetworkNumber("Shooter/Lookup/Speed1_RPM", 1500.0);

  // Data point 2
  public static final LoggedNetworkNumber LOOKUP_DISTANCE_2 =
      new LoggedNetworkNumber("Shooter/Lookup/Distance2_meters", 3.0);
  public static final LoggedNetworkNumber LOOKUP_SPEED_2 =
      new LoggedNetworkNumber("Shooter/Lookup/Speed2_RPM", 3000.0);

  // Add more data points here as needed:
  // public static final LoggedNetworkNumber LOOKUP_DISTANCE_3 =
  //     new LoggedNetworkNumber("Shooter/Lookup/Distance3_meters", 5.0);
  // public static final LoggedNetworkNumber LOOKUP_SPEED_3 =
  //     new LoggedNetworkNumber("Shooter/Lookup/Speed3_RPM", 4500.0);

  /**
   * Builds the distance-to-speed lookup table from LoggedNetworkNumbers.
   * TreeMap automatically sorts by distance for interpolation.
   * 
   * @return TreeMap with distance (meters) as key and speed (RPM) as value
   */
  public static TreeMap<Double, Double> buildLookupTable() {
    TreeMap<Double, Double> table = new TreeMap<>();
    table.put(LOOKUP_DISTANCE_1.get(), LOOKUP_SPEED_1.get());
    table.put(LOOKUP_DISTANCE_2.get(), LOOKUP_SPEED_2.get());
    // Add more data points here:
    // table.put(LOOKUP_DISTANCE_3.get(), LOOKUP_SPEED_3.get());
    return table;
  }

  /**
   * Interpolates shooter speed based on distance using the lookup table.
   * Clamps to min/max speeds if distance is outside the table range.
   * 
   * @param distanceMeters Distance to target in meters
   * @return Shooter speed in RPM
   */
  public static double getSpeedForDistance(double distanceMeters) {
    TreeMap<Double, Double> table = buildLookupTable();

    // Find surrounding data points
    Map.Entry<Double, Double> lowerEntry = table.floorEntry(distanceMeters);
    Map.Entry<Double, Double> upperEntry = table.ceilingEntry(distanceMeters);

    // If exact match, return it
    if (lowerEntry.getKey().equals(distanceMeters)) {
      return lowerEntry.getValue();
    }

    double x1 = lowerEntry.getKey();
    double y1 = lowerEntry.getValue();
    double x2 = upperEntry.getKey();
    double y2 = upperEntry.getValue();
    double x = distanceMeters;

    // LINEAR INTERPOLATION (comment/uncomment as needed)
    double speed = y1 + (y2 - y1) * (x - x1) / (x2 - x1);

    // QUADRATIC INTERPOLATION (uncomment to use, requires at least 3 points)
    // For quadratic: y = a + b*x + c*x^2
    // You'll need to fit a parabola through 3 points
    // double speed = a + b * x + c * x * x;

    // CUBIC INTERPOLATION (uncomment to use, requires at least 4 points)
    // For cubic: y = a + b*x + c*x^2 + d*x^3
    // You'll need to fit a cubic curve through 4 points
    // double speed = a + b * x + c * x * x + d * x * x * x;

    if (speed < MIN_FLYWHEEL_SPEED.in(RevolutionsPerSecond) * 60.0) {
      return MIN_FLYWHEEL_SPEED.in(RevolutionsPerSecond) * 60.0;
    } else if (speed > MAX_FLYWHEEL_SPEED.in(RevolutionsPerSecond) * 60.0) {
      return MAX_FLYWHEEL_SPEED.in(RevolutionsPerSecond) * 60.0;
    } else {
      return speed;
    } 
  }

  // Logs all of the ShooterConstants into Advantage Kit.
  static {
    Logger.recordOutput("Constants/Shooter/FLYWHEEL_GEAR_RATIO", FLYWHEEL_GEAR_RATIO);
    Logger.recordOutput("Constants/Shooter/FLYWHEEL_SPEED_DEADBAND", FLYWHEEL_PID_TOLERANCE);
    Logger.recordOutput(
        "Constants/Shooter/SHOOTER_FLYWHEEL_SMART_CURRENT_LIMIT", SHOOTER_FLYWHEEL_SMART_CURRENT_LIMIT);
    Logger.recordOutput(
        "Constants/Shooter/SHOOTER_FLYWHEEL_SECONDARY_CURRENT_LIMIT", SHOOTER_FLYWHEEL_SECONDARY_CURRENT_LIMIT);
    Logger.recordOutput("Constants/Shooter/SHOOTER_FLYWHEEL_INVERTED", SHOOTER_FLYWHEEL_INVERTED);
    Logger.recordOutput("Constants/Shooter/FLYWHEEL_GEAR_RATIO", FLYWHEEL_GEAR_RATIO);
  }
}
