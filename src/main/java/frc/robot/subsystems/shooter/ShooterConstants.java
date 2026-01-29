package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterConstants {

  // ==================== MECHANICAL CONSTANTS ====================

  /** Gear ratio from motor to flywheel (motor rotations per flywheel rotation) */
  public static final double FLYWHEEL_GEAR_RATIO = 1.0;

  /** Flywheel diameter (4" urethane wheel) */
  public static final Distance FLYWHEEL_DIAMETER = Inches.of(4.0);

  /**
   * Moment of inertia for the flywheel. Estimated for 4" urethane wheel + custom aluminum flywheel.
   * A typical 4" wheel + aluminum disc is approximately 0.003-0.005 kg⋅m²
   */
  public static final MomentOfInertia FLYWHEEL_MOI = KilogramSquareMeters.of(0.0004);

  /** Motor type for the flywheel (1x NEO) */
  public static final DCMotor FLYWHEEL_MOTOR = DCMotor.getNEO(1);

  /** Motor inversion */
  public static final boolean SHOOTER_FLYWHEEL_INVERTED = false;

  // ==================== VELOCITY LIMITS ====================

  public static final AngularVelocity MIN_FLYWHEEL_SPEED =
      RevolutionsPerSecond.of(100 / 60.0); // 100 RPM

  public static final AngularVelocity MAX_FLYWHEEL_SPEED =
      RevolutionsPerSecond.of(5600 / 60.0); // 5600 RPM

  // ==================== ACCELERATION LIMITS ====================

  public static final AngularAcceleration MAX_FLYWHEEL_ACCEL =
      RevolutionsPerSecond.per(Second).of(5600 / 60.0); // 5600 RPM

  // ==================== PID CONSTANTS (REAL ROBOT) ====================

  /** Real robot PID values - tuned for actual hardware (on motor controller) */
  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_PID_KP_REAL =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Real/kP", 0.000500);

  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_PID_KI_REAL =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Real/kI", 0.0);
  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_PID_KD_REAL =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Real/kD", 0.0000);
  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_PID_FF_REAL =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Real/kFF", 0.000100);

  // ==================== FEEDFORWARD CONSTANTS (REAL ROBOT) ====================

  /** Static friction voltage (voltage to overcome friction) */
  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_KS_REAL =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_FF/Real/kS", 0.18554);

  /** Velocity feedforward constant (Volts per RPM) */
  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_KV_REAL =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_FF/Real/kV", 0.002);

  // ==================== FEEDFORWARD CONSTANTS (SIMULATION) ====================
  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_KS_SIM =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_FF/Sim/kS", 0.0);
  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_KV_SIM =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_FF/Sim/kV", 0.0018);

  // ==================== PID CONSTANTS (SIMULATION) ====================

  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_PID_KP_SIM =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Sim/kP", 0.0001);
  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_PID_KI_SIM =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Sim/kI", 0.0);
  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_PID_KD_SIM =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Sim/kD", 0.0);
  public static final LoggedNetworkNumber SHOOTER_FLYWHEEL_PID_FF_SIM =
      new LoggedNetworkNumber("Shooter/FLYWHEEL_PID/Sim/kV", 0.0);

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

  /** Returns the appropriate feedforward kS based on current mode */
  public static double getFlywheelKS() {
    return Constants.currentMode == Constants.Mode.SIM
        ? SHOOTER_FLYWHEEL_KS_SIM.get()
        : SHOOTER_FLYWHEEL_KS_REAL.get();
  }

  /** Returns the appropriate feedforward kV based on current mode */
  public static double getFlywheelKV() {
    return Constants.currentMode == Constants.Mode.SIM
        ? SHOOTER_FLYWHEEL_KV_SIM.get()
        : SHOOTER_FLYWHEEL_KV_REAL.get();
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

  /**
   * Distance-based shooting lookup table.
   *
   * <p>Maps distance to target (meters) → required shooter speed (RPM)
   *
   * <p>Characterize your shooter by:
   *
   * <ol>
   *   <li>Shooting from various known distances
   *   <li>Recording the RPM needed for successful shots
   *   <li>Adding those data points here using Meters.of() and RPM.of()
   * </ol>
   *
   * <p>InterpolatingDoubleTreeMap automatically interpolates between points for any distance.
   */
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SPEED_MAP =
      new InterpolatingDoubleTreeMap();

  static {
    // Add characterization data points using the units library for clarity
    DISTANCE_TO_SPEED_MAP.put(Meters.of(1.0).in(Meters), RPM.of(1000.0).in(RPM));
    DISTANCE_TO_SPEED_MAP.put(Meters.of(3.0).in(Meters), RPM.of(1500.0).in(RPM));
    // Add more data points as you characterize:
    // DISTANCE_TO_SPEED_MAP.put(Meters.of(5.0).in(Meters), RPM.of(4500.0).in(RPM));
  }

  // ==================== FUEL SIM CONSTANTS ====================

  /**
   * FuelSim constants for shooter trajectory visualization.
   *
   * <p>These constants define the shooter's physical position on the robot and launch parameters.
   * They are used by the FuelSimVisualizer to calculate realistic projectile trajectories.
   *
   * <p><b>Coordinate System:</b> Robot-relative coordinates with origin at robot center.
   *
   * <ul>
   *   <li>X-axis: forward (positive ahead of robot)
   *   <li>Y-axis: left/right (positive to left)
   *   <li>Z-axis: up (positive above ground)
   * </ul>
   */

  /** Height of the shooter exit point above the ground. 18" = 0.457 m */
  public static final Distance SHOOTER_HEIGHT_FROM_GROUND = Inches.of(18.0);

  /** Forward offset of shooter from robot center. 8" ahead of center. */
  public static final Distance SHOOTER_FORWARD_OFFSET = Inches.of(8.0);

  /** Side offset of shooter from robot center. Centered left/right = 0.0. */
  public static final Distance SHOOTER_SIDE_OFFSET = Inches.of(0.0);

  /** Fixed hood angle from horizontal. 70° launch angle for the fuel trajectory. */
  public static final Angle SHOOTER_HOOD_ANGLE = Degrees.of(70.0);

  /**
   * Minimum flywheel RPM threshold to trigger fuel launch visualization. Below this speed, no fuel
   * will be visualized as launching.
   */
  public static final AngularVelocity SHOOTER_RPM_THRESHOLD_FOR_LAUNCH = RPM.of(200.0);

  /**
   * Time between consecutive fuel launches. Limits visualization rate to one fuel every 0.5
   * seconds.
   */
  public static final Time SHOOTER_LAUNCH_INTERVAL = Seconds.of(0.5);

  /**
   * Diameter of the shooter wheel for velocity conversion. Uses the same 4" wheel as
   * FLYWHEEL_DIAMETER.
   */
  public static final Distance SHOOTER_WHEEL_DIAMETER = FLYWHEEL_DIAMETER;

  /** Maximum number of fuel pieces the robot can hold. */
  public static final int MAX_HOPPER_CAPACITY = 30;

  /** Number of fuel pieces robot starts with when enabled. */
  public static final int STARTING_FUEL_COUNT = 8;

  /** Number of points used to render the trajectory visualization. */
  public static final int TRAJECTORY_VISUALIZATION_POINTS = 50;

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

    // Log FuelSim constants
    Logger.recordOutput(
        "Constants/Shooter/FuelSim/HeightFromGround_m", SHOOTER_HEIGHT_FROM_GROUND.in(Meters));
    Logger.recordOutput(
        "Constants/Shooter/FuelSim/ForwardOffset_m", SHOOTER_FORWARD_OFFSET.in(Meters));
    Logger.recordOutput("Constants/Shooter/FuelSim/SideOffset_m", SHOOTER_SIDE_OFFSET.in(Meters));
    Logger.recordOutput("Constants/Shooter/FuelSim/HoodAngle_deg", SHOOTER_HOOD_ANGLE.in(Degrees));
    Logger.recordOutput(
        "Constants/Shooter/FuelSim/RPMThreshold", SHOOTER_RPM_THRESHOLD_FOR_LAUNCH.in(RPM));
    Logger.recordOutput(
        "Constants/Shooter/FuelSim/LaunchInterval_s", SHOOTER_LAUNCH_INTERVAL.in(Seconds));
    Logger.recordOutput(
        "Constants/Shooter/FuelSim/WheelDiameter_m", SHOOTER_WHEEL_DIAMETER.in(Meters));
    Logger.recordOutput("Constants/Shooter/FuelSim/MaxHopperCapacity", MAX_HOPPER_CAPACITY);
    Logger.recordOutput("Constants/Shooter/FuelSim/StartingFuelCount", STARTING_FUEL_COUNT);
    Logger.recordOutput(
        "Constants/Shooter/FuelSim/TrajectoryPoints", TRAJECTORY_VISUALIZATION_POINTS);
  }
}
