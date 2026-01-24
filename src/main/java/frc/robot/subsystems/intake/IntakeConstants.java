package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeConstants {

  // ==================== HARDWARE CONFIGURATION ====================

  /** Absolute encoder zero position (intake fully stowed position) */
  public static final Rotation2d PIVOT_ZERO_ROTATION = new Rotation2d(Degrees.of(-140));

  /** Absolute encoder configuration */
  public static final boolean PIVOT_ENCODER_INVERTED = true;

  public static final double PIVOT_ENCODER_POSITION_FACTOR = 2 * Math.PI; // Rotations -> Radians
  public static final double PIVOT_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  /** Motor inversions */
  public static final boolean INTAKE_RIGHT_PIVOT_INVERTED = true;

  public static final boolean INTAKE_ROLLER_INVERTED = false;

  // ==================== MECHANICAL CONSTANTS ====================

  /** Gear ratios - motor rotations per mechanism rotation */
  public static final double PIVOT_GEAR_RATIO = 16.0; // 16:1 reduction

  public static final double ROLLER_GEAR_RATIO = 1.0; // Direct drive

  /** Arm length from pivot point to center of mass (13 inches) */
  public static final Distance ARM_LENGTH = Inches.of(13.0);

  /**
   * Moment of inertia for the intake arm. Calculated for ~15 lbs (6.8 kg) concentrated at end of
   * 13" (0.33m) arm. MOI = m * r^2 = 6.8 * 0.33^2 ≈ 0.74 kg⋅m²
   */
  public static final MomentOfInertia PIVOT_MOI = KilogramSquareMeters.of(0.74);

  /** Angle limits - Zero degrees is stowed (up), positive is towards ground */
  public static final double MIN_INTAKE_ANGLE_DEGREES = -1.0;

  public static final double MAX_INTAKE_ANGLE_DEGREES = 91.0;
  public static final double STARTING_ANGLE_DEGREES = 0.0; // Starts stowed

  /** Motor type for the pivot (2x NEO in leader-follower) */
  public static final DCMotor PIVOT_MOTOR = DCMotor.getNEO(2);

  /** Motor type for the roller (1x NEO) */
  public static final DCMotor ROLLER_MOTOR = DCMotor.getNEO(1);

  // ==================== POSITION SETPOINTS ====================

  /**
   * Position setpoints in degrees. Larger angle is towards the ground, smaller angle is towards the
   * robot.
   */
  public static final LoggedNetworkNumber POSITION_PICKUP =
      new LoggedNetworkNumber("Intake/Position/Pickup", 88);

  public static final LoggedNetworkNumber POSITION_STOW =
      new LoggedNetworkNumber("Intake/Position/Stow", 0);

  // ==================== PID CONSTANTS (REAL ROBOT) ====================

  /** Real robot PID values - tuned for actual hardware */
  public static final LoggedNetworkNumber PIVOT_PID_KP_REAL =
      new LoggedNetworkNumber("Intake/Pivot/PID/Real/kP", 0.03);

  public static final LoggedNetworkNumber PIVOT_PID_KI_REAL =
      new LoggedNetworkNumber("Intake/Pivot/PID/Real/kI", 0.0);
  public static final LoggedNetworkNumber PIVOT_PID_KD_REAL =
      new LoggedNetworkNumber("Intake/Pivot/PID/Real/kD", 0.0);

  /**
   * Feedforward voltage multiplied by cos(angle) to compensate for gravity. When horizontal (0°),
   * full feedforward applies. When vertical (90°), no feedforward needed.
   */
  public static final double INTAKE_PIVOT_FEEDFORWARD_REAL = 0.0;

  // ==================== PID CONSTANTS (SIMULATION) ====================

  /** Simulation PID values - tuned for physics simulation */
  public static final LoggedNetworkNumber PIVOT_PID_KP_SIM =
      new LoggedNetworkNumber("Intake/Pivot/PID/Sim/kP", 0.2);

  public static final LoggedNetworkNumber PIVOT_PID_KI_SIM =
      new LoggedNetworkNumber("Intake/Pivot/PID/Sim/kI", 0.0002);
  public static final LoggedNetworkNumber PIVOT_PID_KD_SIM =
      new LoggedNetworkNumber("Intake/Pivot/PID/Sim/kD", 0.0015);

  /** Simulation feedforward for gravity compensation */
  public static final double INTAKE_PIVOT_FEEDFORWARD_SIM = 2;

  // ==================== MODE-SELECTED CONSTANTS ====================

  /** Returns the appropriate PID kP based on current mode */
  public static double getPivotKP() {
    return Constants.currentMode == Constants.Mode.SIM
        ? PIVOT_PID_KP_SIM.get()
        : PIVOT_PID_KP_REAL.get();
  }

  /** Returns the appropriate PID kI based on current mode */
  public static double getPivotKI() {
    return Constants.currentMode == Constants.Mode.SIM
        ? PIVOT_PID_KI_SIM.get()
        : PIVOT_PID_KI_REAL.get();
  }

  /** Returns the appropriate PID kD based on current mode */
  public static double getPivotKD() {
    return Constants.currentMode == Constants.Mode.SIM
        ? PIVOT_PID_KD_SIM.get()
        : PIVOT_PID_KD_REAL.get();
  }

  /** Returns the appropriate feedforward based on current mode */
  public static double getPivotFeedforward() {
    return Constants.currentMode == Constants.Mode.SIM
        ? INTAKE_PIVOT_FEEDFORWARD_SIM
        : INTAKE_PIVOT_FEEDFORWARD_REAL;
  }

  // ==================== CURRENT LIMITS ====================

  public static final int INTAKE_PIVOT_SMART_CURRENT_LIMIT = 40;
  public static final double INTAKE_PIVOT_SECONDARY_CURRENT_LIMIT = 55;
  public static final int INTAKE_ROLLER_SMART_CURRENT_LIMIT = 50;
  public static final double INTAKE_ROLLER_SECONDARY_CURRENT_LIMIT = 60;

  // ==================== ROLLER SETTINGS ====================

  public static final LoggedNetworkNumber INTAKE_ACQUIRE_SPEED =
      new LoggedNetworkNumber("Intake/Roller/Aquire Speed", 1);

  public static final LoggedNetworkNumber INTAKE_CURRENT_CUTOFF =
      new LoggedNetworkNumber("Intake/Roller/Current Cutoff", 40);

  // ==================== SOFTWARE SETTINGS ====================

  /** Deadband for considering pivot "at position" in degrees */
  public static final double INTAKE_PIVOT_DEADBAND = 2.0;

  // ==================== VISUALIZATION CONSTANTS ====================

  /**
   * 3D offset from robot origin to intake pivot point. TODO: Update these values based on CAD or
   * measurements. X = forward/back, Y = up/down, Z = left/right (meters)
   */
  public static final Translation3d VISUALIZATION_OFFSET =
      new Translation3d(Inches.of(8).in(Meters), Inches.of(8).in(Meters), Inches.of(0).in(Meters));

  public static final Rotation3d VISUALIZATION_ROTATION = new Rotation3d(0.0, 0.0, Math.PI);

  /** Visualization arm length in meters for Mechanism2d display */
  public static final double VISUALIZATION_ARM_LENGTH = ARM_LENGTH.in(Meters);

  // ==================== LOGGING ====================

  static {
    Logger.recordOutput("Constants/Intake/MIN_INTAKE_ANGLE", MIN_INTAKE_ANGLE_DEGREES);
    Logger.recordOutput("Constants/Intake/MAX_INTAKE_ANGLE", MAX_INTAKE_ANGLE_DEGREES);
    Logger.recordOutput("Constants/Intake/PIVOT_GEAR_RATIO", PIVOT_GEAR_RATIO);
    Logger.recordOutput("Constants/Intake/ROLLER_GEAR_RATIO", ROLLER_GEAR_RATIO);
    Logger.recordOutput("Constants/Intake/ARM_LENGTH_METERS", ARM_LENGTH.in(Meters));
    Logger.recordOutput("Constants/Intake/PIVOT_MOI", PIVOT_MOI.in(KilogramSquareMeters));
    Logger.recordOutput("Constants/Intake/INTAKE_PIVOT_DEADBAND", INTAKE_PIVOT_DEADBAND);
    Logger.recordOutput(
        "Constants/Intake/INTAKE_PIVOT_SMART_CURRENT_LIMIT", INTAKE_PIVOT_SMART_CURRENT_LIMIT);
    Logger.recordOutput(
        "Constants/Intake/INTAKE_PIVOT_SECONDARY_CURRENT_LIMIT",
        INTAKE_PIVOT_SECONDARY_CURRENT_LIMIT);
    Logger.recordOutput(
        "Constants/Intake/INTAKE_ROLLER_SMART_CURRENT_LIMIT", INTAKE_ROLLER_SMART_CURRENT_LIMIT);
    Logger.recordOutput(
        "Constants/Intake/INTAKE_ROLLER_SECONDARY_CURRENT_LIMIT",
        INTAKE_ROLLER_SECONDARY_CURRENT_LIMIT);
  }
}
