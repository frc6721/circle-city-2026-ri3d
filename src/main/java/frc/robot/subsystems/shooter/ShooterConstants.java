package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.Constants;
import java.util.Map;
import java.util.TreeMap;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterConstants {

  // ==================== MECHANICAL CONSTANTS ====================

  /** Gear ratio from motor to flywheel (motor rotations per flywheel rotation) */
  public static final double FLYWHEEL_GEAR_RATIO = 2.0;

  /** Flywheel diameter (4" urethane wheel) */
  public static final Distance FLYWHEEL_DIAMETER = Inches.of(4.0);

  /**
   * Moment of inertia for the flywheel. Estimated for 4" urethane wheel + custom aluminum flywheel.
   * A typical 4" wheel + aluminum disc is approximately 0.003-0.005 kg⋅m²
   */
  public static final MomentOfInertia FLYWHEEL_MOI = KilogramSquareMeters.of(0.004);

  /** Motor type for the flywheel (1x NEO) */
  public static final DCMotor FLYWHEEL_MOTOR = DCMotor.getNEO(1);

  /** Motor inversion */
  public static final boolean SHOOTER_FLYWHEEL_INVERTED = false;

  // ==================== VELOCITY LIMITS ====================

  public static final AngularVelocity MIN_FLYWHEEL_SPEED =
      RevolutionsPerSecond.of(100 / 60.0); // 100 RPM

  public static final AngularVelocity MAX_FLYWHEEL_SPEED =
      RevolutionsPerSecond.of(10000 / 60.0); // 10000 RPM

  // ==================== PID CONSTANTS (REAL ROBOT) ====================

  /** Real robot PID values - tuned for actual hardware (on motor controller) */
  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_PID_KP_REAL =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Real/kP", 0.0006);

  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_PID_KI_REAL =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Real/kI", 0.00000006);
  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_PID_KD_REAL =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Real/kD", 0.0005);
  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_PID_FF_REAL =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Real/kFF", 0.0000008);

  // ==================== PID CONSTANTS (SIMULATION) ====================

  /** Simulation PID values - tuned for physics simulation */
  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_PID_KP_SIM =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Sim/kP", 0.001);

  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_PID_KI_SIM =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Sim/kI", 0.0);
  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_PID_KD_SIM =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Sim/kD", 0.0);
  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_PID_FF_SIM =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Sim/kFF", 0.00018);

  // ==================== MODE-SELECTED CONSTANTS ====================

  /** Returns the appropriate PID kP based on current mode */
  public static double getFlywheelKP() {
    return Constants.currentMode == Constants.Mode.SIM
        ? SHOOTER_FLYWHEEL_PID_KP_SIM.get()
        : SHOOTER_FLYWHEEL_PID_KP_REAL.get();
  }

  /** Returns the appropriate PID kI based on current mode */
  public static double getFlywheelKI() {
    return Constants.currentMode == Constants.Mode.SIM
        ? SHOOTER_FLYWHEEL_PID_KI_SIM.get()
        : SHOOTER_FLYWHEEL_PID_KI_REAL.get();
  }

  /** Returns the appropriate PID kD based on current mode */
  public static double getFlywheelKD() {
    return Constants.currentMode == Constants.Mode.SIM
        ? SHOOTER_FLYWHEEL_PID_KD_SIM.get()
        : SHOOTER_FLYWHEEL_PID_KD_REAL.get();
  }

  /** Returns the appropriate PID FF based on current mode */
  public static double getFlywheelFF() {
    return Constants.currentMode == Constants.Mode.SIM
        ? SHOOTER_FLYWHEEL_PID_FF_SIM.get()
        : SHOOTER_FLYWHEEL_PID_FF_REAL.get();
  }

  // ==================== CURRENT LIMITS ====================

  public static final int SHOOTER_FLYWHEEL_SMART_CURRENT_LIMIT = 100;
  public static final double SHOOTER_FLYWHEEL_SECONDARY_CURRENT_LIMIT = 100;

  // ==================== SOFTWARE SETTINGS ====================

  /** Tolerance as a percentage (0.05 = 5%) for determining if flywheel is at target speed */
  public static final double FLYWHEEL_PID_TOLERANCE = 0.05;

  /** Idle duty cycle when shooter is not actively shooting */
  public static final double SHOOTER_IDLE_DUTY_CYCLE_OUTPUT = 0.0;

  // ==================== VISUALIZATION CONSTANTS ====================

  /**
   * 3D offset from robot origin to flywheel center. TODO: Update these values based on CAD or
   * measurements. X = forward/back, Y = up/down, Z = left/right (meters)
   */
  public static final Translation3d VISUALIZATION_OFFSET =
      new Translation3d(
          Inches.of(26 / 2).in(Meters), Inches.of(20).in(Meters), Inches.of(6).in(Meters));

  /** 3D rotation offset for visualization orientation */
  public static final Rotation3d VISUALIZATION_ROTATION = new Rotation3d(0.0, 0.0, 0.0);

  /** Visualization flywheel radius in meters for Mechanism2d display */
  public static final double VISUALIZATION_FLYWHEEL_RADIUS = FLYWHEEL_DIAMETER.in(Meters) / 2.0;

  // ==================== DISTANCE TO SPEED LOOKUP TABLE ====================

  /** Data point 1 - Distance units: meters, Speed units: RPM */
  public static final LoggedNetworkNumber LOOKUP_DISTANCE_1 =
      new LoggedNetworkNumber("Shooter/Lookup/Distance1_meters", 1.0);

  public static final LoggedNetworkNumber LOOKUP_SPEED_1 =
      new LoggedNetworkNumber("Shooter/Lookup/Speed1_RPM", 1500.0);

  /** Data point 2 */
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
   * Builds the distance-to-speed lookup table from LoggedNetworkNumbers. TreeMap automatically
   * sorts by distance for interpolation.
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
   * Interpolates shooter speed based on distance using the lookup table. Clamps to min/max speeds
   * if distance is outside the table range.
   *
   * @param distanceMeters Distance to target in meters
   * @return Shooter speed in RPM
   */
  public static double getSpeedForDistance(double distanceMeters) {
    TreeMap<Double, Double> table = buildLookupTable();

    // Find surrounding data points
    Map.Entry<Double, Double> lowerEntry = table.floorEntry(distanceMeters);
    Map.Entry<Double, Double> upperEntry = table.ceilingEntry(distanceMeters);

    // Handle edge cases
    if (lowerEntry == null) {
      return table.firstEntry().getValue();
    }
    if (upperEntry == null) {
      return table.lastEntry().getValue();
    }

    // If exact match, return it
    if (lowerEntry.getKey().equals(distanceMeters)) {
      return lowerEntry.getValue();
    }

    double x1 = lowerEntry.getKey();
    double y1 = lowerEntry.getValue();
    double x2 = upperEntry.getKey();
    double y2 = upperEntry.getValue();
    double x = distanceMeters;

    // LINEAR INTERPOLATION
    double speed = y1 + (y2 - y1) * (x - x1) / (x2 - x1);

    // Clamp to min/max speeds
    if (speed < MIN_FLYWHEEL_SPEED.in(RevolutionsPerSecond) * 60.0) {
      return MIN_FLYWHEEL_SPEED.in(RevolutionsPerSecond) * 60.0;
    } else if (speed > MAX_FLYWHEEL_SPEED.in(RevolutionsPerSecond) * 60.0) {
      return MAX_FLYWHEEL_SPEED.in(RevolutionsPerSecond) * 60.0;
    } else {
      return speed;
    }
  }

  // ==================== LOGGING ====================

  static {
    Logger.recordOutput("Constants/Shooter/FLYWHEEL_GEAR_RATIO", FLYWHEEL_GEAR_RATIO);
    Logger.recordOutput("Constants/Shooter/FLYWHEEL_MOI", FLYWHEEL_MOI.in(KilogramSquareMeters));
    Logger.recordOutput("Constants/Shooter/FLYWHEEL_SPEED_DEADBAND", FLYWHEEL_PID_TOLERANCE);
    Logger.recordOutput(
        "Constants/Shooter/SHOOTER_FLYWHEEL_SMART_CURRENT_LIMIT",
        SHOOTER_FLYWHEEL_SMART_CURRENT_LIMIT);
    Logger.recordOutput(
        "Constants/Shooter/SHOOTER_FLYWHEEL_SECONDARY_CURRENT_LIMIT",
        SHOOTER_FLYWHEEL_SECONDARY_CURRENT_LIMIT);
    Logger.recordOutput("Constants/Shooter/SHOOTER_FLYWHEEL_INVERTED", SHOOTER_FLYWHEEL_INVERTED);
  }
}
