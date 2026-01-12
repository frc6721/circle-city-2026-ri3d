package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

public class ClimberConstants {
  /*************************
   *
   * CLIMBER MOTOR SETTINGS
   *
   *************************/
  public static final int CLIMBER_SMART_CURRENT_LIMIT = 40;

  public static final double CLIMBER_SECONDARY_CURRENT_LIMIT = 55;
  public static final boolean CLIMBER_INVERTED = false;

  // Logs all of the ClimberConstants into Advantage Kit.
  static {
    Logger.recordOutput(
        "Constants/Climber/CLIMBER_SMART_CURRENT_LIMIT", CLIMBER_SMART_CURRENT_LIMIT);
    Logger.recordOutput(
        "Constants/Climber/CLIMBER_SECONDARY_CURRENT_LIMIT", CLIMBER_SECONDARY_CURRENT_LIMIT);
    Logger.recordOutput("Constants/Climber/CLIMBER_INVERTED", CLIMBER_INVERTED);
  }
}
