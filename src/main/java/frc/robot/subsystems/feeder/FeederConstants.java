package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

public class FeederConstants {
  /*********************
   *
   * FEEDER SOFTWARE CONSTANTS
   *
   **********************/
  public static final int FEEDER_MOTOR_SMART_CURRENT_LIMIT = 40;

  public static final double FEEDER_MOTOR_SECONDARY_CURRENT_LIMIT = 55;
  public static final boolean FEEDER_MOTOR_INVERTED = false;

  // Logs all of the FeederConstants into Advantage Kit.
  static {
    Logger.recordOutput(
        "Constants/Feeder/FEEDER_MOTOR_SMART_CURRENT_LIMIT", FEEDER_MOTOR_SMART_CURRENT_LIMIT);
    Logger.recordOutput(
        "Constants/Feeder/FEEDER_MOTOR_SECONDARY_CURRENT_LIMIT",
        FEEDER_MOTOR_SECONDARY_CURRENT_LIMIT);
    Logger.recordOutput("Constants/Feeder/FEEDER_MOTOR_INVERTED", FEEDER_MOTOR_INVERTED);
  }
}
