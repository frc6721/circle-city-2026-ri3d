package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeConstants {
  /*************************
   *
   * INTAKE HARDWARE CONSTANTS
   *
   ************************/

  // Absolute encoder zero position (intake fully stowed position)
  public static final Rotation2d PIVOT_ZERO_ROTATION = new Rotation2d(-1.256);

  /************************
   *
   * INTAKE PIVOT PID CONSTANTS
   *
   *************************/
  public static final LoggedNetworkNumber PIVOT_PID_KP =
      new LoggedNetworkNumber("Intake/Pivot/PID/kP", 0.1);

  public static final LoggedNetworkNumber PIVOT_PID_KI =
      new LoggedNetworkNumber("Intake/Pivot/PID/kI", 0.000000);
  public static final LoggedNetworkNumber PIVOT_PID_KD =
      new LoggedNetworkNumber("Intake/Pivot/PID/kD", 0.00000);

  /***********************
   *
   * INTAKE POSITION SETPOINTS
   *
   * units: Degrees
   * larger angle is towards the ground. smaller angle is towards the robot
   *
   **********************/
  public static final LoggedNetworkNumber POSITION_PICKUP =
      new LoggedNetworkNumber("Intake/Position/Pickup", 90);

  public static final LoggedNetworkNumber POSITION_STOW =
      new LoggedNetworkNumber("Intake/Position/Stow", 0);

  /*************************
   *
   * INTAKE MECHANICAL CONSTANTS
   *
   ************************/
  // Zero degrees for the intake is defined as the intake is stowed
  public static final double MIN_INTAKE_ANGLE = -1.0;

  public static final double MAX_INTAKE_ANGLE = 91.0;

  // Absolute encoder configuration
  public static final boolean PIVOT_ENCODER_INVERTED = true;
  public static final double PIVOT_ENCODER_POSITION_FACTOR = 2 * Math.PI; // Rotations -> Radians
  public static final double PIVOT_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  /***************************
   *
   * INTAKE ROLLER SETTINGS
   *
   **************************/
  public static final LoggedNetworkNumber INTAKE_ACQUIRE_SPEED =
      new LoggedNetworkNumber("Intake/Roller/Aquire Speed", 0.8);

  public static final LoggedNetworkNumber INTAKE_CURRENT_CUTOFF =
      new LoggedNetworkNumber("Intake/Roller/Current Cutoff", 40);

  /*********************
   *
   * INTAKE SOFTWARE CONSTANTS
   *
   **********************/
  public static final double INTAKE_PIVOT_DEADBAND = 10;

  public static final int INTAKE_PIVOT_SMART_CURRENT_LIMIT = 40;
  public static final double INTAKE_PIVOT_SECONDARY_CURRENT_LIMIT = 55;
  public static final int INTAKE_ROLLER_SMART_CURRENT_LIMIT = 50;
  public static final double INTAKE_ROLLER_SECONDARY_CURRENT_LIMIT = 60;
  public static final boolean INTAKE_RIGHT_PIVOT_INVERTED = false;
  public static final boolean INTAKE_LEFT_PIVOT_INVERTED = true; // Follower, inverted
  public static final boolean INTAKE_ROLLER_INVERTED = false;
  // Gear ratio from motor to end effector
  public static final double INTAKE_PIVOT_CONVERSION_FACTOR = 1;
  public static final double INTAKE_ROLLER_CONVERSION_FACTOR = 1;

  // Logs all of the IntakeConstants into Advantage Kit.
  static {
    Logger.recordOutput("Constants/Intake/MIN_INTAKE_ANGLE", MIN_INTAKE_ANGLE);
    Logger.recordOutput("Constants/Intake/MAX_INTAKE_ANGLE", MAX_INTAKE_ANGLE);

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
