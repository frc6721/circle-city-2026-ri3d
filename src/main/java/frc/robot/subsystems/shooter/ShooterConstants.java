package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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
